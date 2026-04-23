"""
ACC Project - Raspberry Pi CAN Interface Module
================================================
담당: Raspi CAN 모듈 (DBC-driven, cantools 기반)

역할:
  - HMI/Fusion 모듈로부터 데이터를 받아 ECU로 CAN 송신
  - ECU로부터 CAN 수신하여 HMI 모듈에 제공 (폴링)
  - Heartbeat 감시 및 ERR 처리

진실의 원천:
  CAN ID / 페이로드 레이아웃 / 시그널 범위는 모두 `../ACC-CANDB/acc_db.dbc`
  가 결정한다. 본 모듈은 cantools 로 DBC 를 import 시점에 로드하여 ID 와
  encode/decode 를 위임한다. 수동 bit-shift / byte 조립 금지 — DBC 가 바뀌면
  자동으로 반영되어야 한다.

CAN 메시지 (DBC 2026-04-23 기준):
  TX (SENSOR → ECU):
    0x110 SENSOR_FUSION     전방차량 감지/거리    20ms
    0x111 SENSOR_HEARTBEAT  RPi HB/ERR            20ms
    0x120 VEH_CTRL          brake + accel_pwm     20ms  (SYS006/007, SAF004/015)
    0x510 ACC_CTRL          ACC 버튼 + 목표       50ms  (SYS030)
  RX (ECU → SENSOR):
    0x410 ECU_HEARTBEAT     ECU HB/ERR            10ms  (SAF018 ASIL-B)
    0x520 ACC_STATUS        ACC 상태 피드백       50ms  (SYS031)

  * MTR_SPD_FB (0x300 GET_SPD_AVG) RX 는 현재 미구현 — _vehicle_info.current_speed
    는 0.0 에서 갱신되지 않는다. HMI 는 자차속도 표시 위해 추후 RX 추가 필요.

타임아웃:
  SENSOR_FUSION 미갱신 60ms → ERR_SENSOR 설정  (SWR013)
  ECU_HEARTBEAT 미수신 150ms → heartbeat_ok=False

관련 요구사항: SYS006, SYS007, SYS015, SYS025, SYS030, SYS031,
              SWR013, SWR018, SWR019, SWR020, SWR030,
              SAF004, SAF015, SAF018
"""

__version__ = "0.2.0"
__author__ = "Wis3754"

import threading  # TX / RX / HB 병렬 실행
import time
import logging
from pathlib import Path
from typing import Optional

import cantools

from interfaces.acc_status import AccStatus
from interfaces.button_input import ButtonInput
from interfaces.pedal_input import PedalInput
from interfaces.acc_setting import AccSetting
from interfaces.fusion_data import FusionData
from interfaces.acc_info import AccInfo
from interfaces.vehicle_info import VehicleInfo
from interfaces.ecu_status import EcuStatus

try:
    import can

    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False

logger = logging.getLogger(__name__)

# ── DBC 로드 ─────────────────────────────────────────────────────────────────
# 상대경로: ACC-RPI/acc_can/__init__.py → parents[2]=/mnt/c/acc → ACC-CANDB/
_DBC_PATH = Path(__file__).resolve().parents[2] / "ACC-CANDB" / "acc_db.dbc"
_db = cantools.database.load_file(str(_DBC_PATH))   # 실패 시 ImportError 전파


def _msg(name: str):
    """DBC 메시지 객체 조회 (이름 오타 시 KeyError)."""
    return _db.get_message_by_name(name)


def _id(name: str) -> int:
    """DBC 메시지의 CAN arbitration ID."""
    return _msg(name).frame_id


# ── CAN Message IDs (DBC 조회 — 하드코딩 금지) ──────────────────────────────
MSG_ID_SENSOR_FUSION    = _id("SENSOR_FUSION")      # 0x110
MSG_ID_SENSOR_HEARTBEAT = _id("SENSOR_HEARTBEAT")   # 0x111
MSG_ID_VEH_CTRL         = _id("VEH_CTRL")           # 0x120
MSG_ID_ACC_CTRL         = _id("ACC_CTRL")           # 0x510
MSG_ID_ACC_STATUS       = _id("ACC_STATUS")         # 0x520
MSG_ID_ECU_HEARTBEAT    = _id("ECU_HEARTBEAT")      # 0x410

# ── TX 주기 ──────────────────────────────────────────────────────────────────
TX_PERIOD_20MS_SEC = 0.020  # VEH_CTRL, SENSOR_FUSION, SENSOR_HEARTBEAT
TX_PERIOD_50MS_SEC = 0.050  # ACC_CTRL
TX_POLL_SEC        = 0.005  # TX 루프 폴링 간격 (5ms)
GUI_REFRESH_MS     = 50     # HMI 폴링 주기 (참고용)

# ── Heartbeat 타임아웃 ──────────────────────────────────────────────────────
HB_TIMEOUT_SEC = 0.15  # 150ms (15주기 * 10ms) 미수신 시 heartbeat_ok=False


class CanInterface:
    """
    Raspberry Pi CAN 통신 모듈 (DBC-driven).

    사용법:
        can_mod = CanInterface(channel='can0', bustype='socketcan')
        can_mod.start()
        ...
        can_mod.send_button_input(ButtonInput(btn_acc_off=True, ...))
        info = can_mod.get_acc_info()
        ...
        can_mod.stop()
    """

    def __init__(self, channel: str = "can0", bustype: str = "socketcan"):
        self._channel = channel
        self._bustype = bustype
        self._bus: Optional[object] = None  # 연결 전
        self._running = False  # 스레드 플래그

        # ── TX 버퍼 ──────────────────────────────────────────────────────────
        self._lock = threading.Lock()  # RX/TX/HMI 간 동시 접근 보호

        # button_input: Edge Trigger — TX 후 자동 클리어
        self._button_input = ButtonInput(
            btn_acc_off=False,
            btn_acc_set=False,
            btn_acc_res=False,
            btn_acc_cancel=False,
        )

        # pedal_input: 연속값 — 최신값 유지
        self._pedal_input = PedalInput(
            brake=False,
            accel_pwm=0,  # DBC SET_ACCEL_PWM int8 (-128~127)
        )

        # acc_setting: 최신값 유지
        self._acc_setting = AccSetting(
            set_speed=0,        # DBC SET_ACC_SPD int16 (-500~500 cm/s)
            distance_level=3,   # DBC SET_ACC_LVL (1/2/3)
        )

        # fusion_data: Fusion 모듈이 갱신
        self._fusion_data = FusionData(
            detected=False,
            distance=0,  # DBC VEH_DIST uint16 (0~12000 mm)
        )
        self._fusion_last_update = 0.0

        # ── RX 버퍼 ──────────────────────────────────────────────────────────
        self._acc_info = AccInfo(
            status=AccStatus.OFF.value,
            set_speed=0,
            distance_level=3,
        )

        # NOTE: MTR_SPD_FB RX 미구현 — current_speed 는 0.0 고정.
        # 자차속도 표시 필요 시 MTR_SPD_FB (0x300) 의 GET_SPD_AVG RX 추가.
        self._vehicle_info = VehicleInfo(current_speed=0.0)

        self._ecu_status = EcuStatus(
            heartbeat_ok=False,
            error_code=0,
        )

        self._hb_last_rx = 0.0
        self._hb_sensor_counter = 0

        # ── 스레드 ────────────────────────────────────────────────────────────
        self._tx_thread: Optional[threading.Thread] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._hb_thread: Optional[threading.Thread] = None

    # =========================================================================
    # 생명주기
    # =========================================================================

    def start(self) -> None:
        """CAN 버스 초기화 + TX/RX/HB 스레드 시작."""
        if self._running:
            logger.warning("CanInterface already running.")
            return

        if CAN_AVAILABLE:
            try:
                self._bus = can.interface.Bus(
                    channel=self._channel, bustype=self._bustype
                )
                logger.info(f"CAN bus opened: {self._channel} ({self._bustype})")
            except Exception as e:
                logger.error(f"CAN bus open failed: {e}. Running in simulation mode.")
                self._bus = None
        else:
            logger.warning("python-can not installed. Running in simulation mode.")
            self._bus = None

        self._running = True
        self._hb_last_rx = time.time()

        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True, name="CAN_TX")
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True, name="CAN_RX")
        self._hb_thread = threading.Thread(target=self._hb_watchdog, daemon=True, name="CAN_HB")

        self._tx_thread.start()
        self._rx_thread.start()
        self._hb_thread.start()
        logger.info("CanInterface started.")

    def stop(self) -> None:
        """TX 타이머/RX 스레드 종료 + CAN 버스 해제."""
        self._running = False

        for t in (self._tx_thread, self._rx_thread, self._hb_thread):
            if t and t.is_alive():
                t.join(timeout=1.0)

        if self._bus:
            try:
                self._bus.shutdown()
            except Exception as e:
                logger.error(f"CAN bus shutdown error: {e}")
            self._bus = None

        logger.info("CanInterface stopped.")

    def is_connected(self) -> bool:
        """CAN 버스 연결 상태. 시뮬레이션 모드면 False."""
        return self._bus is not None

    # =========================================================================
    # HMI → CAN 모듈 (송신용 setter)
    # =========================================================================

    def send_button_input(self, button_input: ButtonInput) -> None:
        """
        버튼 Edge Trigger 입력 전달. 호출: 버튼 클릭 즉시.
        다음 50ms ACC_CTRL TX 주기에 송신 후 자동 클리어.
        같은 버튼이 주기 내 여러 번 눌리면 OR 로 누적 (한번 송신).
        """
        with self._lock:
            if button_input.btn_acc_off is not None:
                self._button_input.btn_acc_off |= bool(button_input.btn_acc_off)
            if button_input.btn_acc_set is not None:
                self._button_input.btn_acc_set |= bool(button_input.btn_acc_set)
            if button_input.btn_acc_res is not None:
                self._button_input.btn_acc_res |= bool(button_input.btn_acc_res)
            if button_input.btn_acc_cancel is not None:
                self._button_input.btn_acc_cancel |= bool(button_input.btn_acc_cancel)

    def send_pedal_input(self, pedal_input: PedalInput) -> None:
        """
        페달 연속값 전달. 호출: 페달값 변경 즉시 (디바운싱 없음).

        Args:
            pedal_input: PedalInput (brake: bool, accel_pwm: -128~127 DBC SET_ACCEL_PWM int8)
        """
        with self._lock:
            self._pedal_input.brake = bool(pedal_input.brake)
            # DBC SET_ACCEL_PWM int8 (-128~127) — 음수=reverse
            self._pedal_input.accel_pwm = max(-128, min(127, int(pedal_input.accel_pwm)))

    def send_acc_setting(self, acc_setting: AccSetting) -> None:
        """
        ACC 설정값 전달 (HMI 에서 계산한 절대값).

        Args:
            acc_setting: AccSetting (set_speed: -500~500 cm/s DBC SET_ACC_SPD int16,
                                     distance_level: 1/2/3)
        """
        with self._lock:
            # DBC SET_ACC_SPD int16 (-500~500 cm/s)
            self._acc_setting.set_speed = max(-500, min(500, int(acc_setting.set_speed)))
            self._acc_setting.distance_level = max(1, min(3, int(acc_setting.distance_level)))

    # =========================================================================
    # Fusion → CAN 모듈
    # =========================================================================

    def update_fusion_data(self, fusion_data: FusionData) -> None:
        """
        Fusion 모듈 산출 전방 차량 데이터 전달. 호출: Fusion 센서 처리 완료 시.

        Args:
            fusion_data: FusionData (detected: bool, distance: 0~12000 mm DBC VEH_DIST uint16)
        """
        with self._lock:
            self._fusion_data.detected = bool(fusion_data.detected)
            # DBC VEH_DIST uint16 (0~12000 mm)
            self._fusion_data.distance = max(0, min(12000, int(fusion_data.distance)))
            self._fusion_last_update = time.time()

    # =========================================================================
    # CAN 모듈 → HMI (수신 데이터 getter)
    # =========================================================================

    def get_acc_info(self) -> AccInfo:
        """ECU 에서 수신한 ACC 상태 반환. HMI 50ms 폴링용."""
        with self._lock:
            return AccInfo(
                status=self._acc_info.status,
                set_speed=self._acc_info.set_speed,
                distance_level=self._acc_info.distance_level,
            )

    def get_vehicle_info(self) -> VehicleInfo:
        """자차 속도 반환. 현재 MTR_SPD_FB RX 미구현 → 항상 0.0."""
        with self._lock:
            return VehicleInfo(current_speed=self._vehicle_info.current_speed)

    def get_ecu_status(self) -> EcuStatus:
        """ECU 통신 상태 + 에러 코드 반환."""
        with self._lock:
            return EcuStatus(
                heartbeat_ok=self._ecu_status.heartbeat_ok,
                error_code=self._ecu_status.error_code,
            )

    # =========================================================================
    # 내부: TX 루프 (multi-rate: 20ms + 50ms)
    # =========================================================================

    def _tx_loop(self) -> None:
        """
        20ms 주기: VEH_CTRL, SENSOR_FUSION, SENSOR_HEARTBEAT
        50ms 주기: ACC_CTRL
        5ms 폴링으로 두 주기 독립 관리 (deadline 시 각자 재스케줄).
        """
        next_20 = time.monotonic()
        next_50 = time.monotonic()
        while self._running:
            now = time.monotonic()
            if now >= next_20:
                self._tx_veh_ctrl()
                self._tx_sensor_fusion()
                self._tx_sensor_heartbeat()
                next_20 += TX_PERIOD_20MS_SEC
            if now >= next_50:
                self._tx_acc_ctrl()
                next_50 += TX_PERIOD_50MS_SEC
            time.sleep(TX_POLL_SEC)

    def _tx_veh_ctrl(self) -> None:
        """0x120 VEH_CTRL (20ms) — brake + accel PWM."""
        with self._lock:
            signals = {
                "SET_ACCEL_PWM": int(self._pedal_input.accel_pwm),
                "BTN_BRAKE":     int(self._pedal_input.brake),
            }
        try:
            data = _msg("VEH_CTRL").encode(signals)
        except Exception as e:
            logger.error(f"VEH_CTRL encode error: {e}")
            return
        self._send_raw(MSG_ID_VEH_CTRL, data)

    def _tx_acc_ctrl(self) -> None:
        """
        0x510 ACC_CTRL (50ms) — ACC 버튼 (edge trigger) + 목표 속도/레벨.
        TX 후 버튼 비트 즉시 클리어 — 한 번 펄스 후 0 유지 (DBC edge trigger 규약).
        """
        with self._lock:
            signals = {
                "BTN_ACC_OFF":    int(self._button_input.btn_acc_off),
                "BTN_ACC_SET":    int(self._button_input.btn_acc_set),
                "BTN_ACC_RES":    int(self._button_input.btn_acc_res),
                "BTN_ACC_CANCEL": int(self._button_input.btn_acc_cancel),
                "SET_ACC_SPD":    int(self._acc_setting.set_speed),
                "SET_ACC_LVL":    int(self._acc_setting.distance_level),
            }
            # Edge trigger: TX 후 클리어
            self._button_input.btn_acc_off = False
            self._button_input.btn_acc_set = False
            self._button_input.btn_acc_res = False
            self._button_input.btn_acc_cancel = False

        try:
            data = _msg("ACC_CTRL").encode(signals)
        except Exception as e:
            logger.error(f"ACC_CTRL encode error: {e}")
            return
        self._send_raw(MSG_ID_ACC_CTRL, data)

    def _tx_sensor_fusion(self) -> None:
        """0x110 SENSOR_FUSION (20ms) — 전방 차량 감지 + 거리(mm)."""
        with self._lock:
            signals = {
                "VEH_DET":  int(self._fusion_data.detected),
                "VEH_DIST": int(self._fusion_data.distance),
            }
        try:
            data = _msg("SENSOR_FUSION").encode(signals)
        except Exception as e:
            logger.error(f"SENSOR_FUSION encode error: {e}")
            return
        self._send_raw(MSG_ID_SENSOR_FUSION, data)

    def _tx_sensor_heartbeat(self) -> None:
        """
        0x111 SENSOR_HEARTBEAT (20ms) — HB 카운터 + ERR 플래그.
        Fusion 데이터 60ms 이상 미갱신 시 ERR_SENSOR=1 (SWR013 타임아웃).
        """
        with self._lock:
            last_upd = self._fusion_last_update

        now = time.time()
        err_sensor = 1 if (now - last_upd > 0.06 and last_upd != 0.0) else 0
        self._hb_sensor_counter = (self._hb_sensor_counter + 1) % 256

        signals = {
            "HB_SENSOR":  self._hb_sensor_counter,
            "ERR_SENSOR": err_sensor,
        }
        try:
            data = _msg("SENSOR_HEARTBEAT").encode(signals)
        except Exception as e:
            logger.error(f"SENSOR_HEARTBEAT encode error: {e}")
            return
        self._send_raw(MSG_ID_SENSOR_HEARTBEAT, data)

    # =========================================================================
    # 내부: RX 루프
    # =========================================================================

    def _rx_loop(self) -> None:
        """ECU 메시지 수신 → 파싱 → 내부 버퍼 갱신."""
        while self._running:
            if self._bus is None:
                time.sleep(0.05)
                continue
            try:
                msg = self._bus.recv(timeout=0.1)
                if msg is None:
                    continue
                if msg.arbitration_id == MSG_ID_ACC_STATUS:
                    self._parse_acc_status(msg.data)
                elif msg.arbitration_id == MSG_ID_ECU_HEARTBEAT:
                    self._parse_ecu_heartbeat(msg.data)
                # MTR_SPD_FB (0x300) RX 는 현재 미구현
            except Exception as e:
                logger.error(f"CAN RX error: {e}")

    def _parse_acc_status(self, data: bytes) -> None:
        """0x520 ACC_STATUS 파싱 (cantools decode — 레이아웃 전부 DBC 에 위임)."""
        try:
            signals = _msg("ACC_STATUS").decode(data)
        except Exception as e:
            logger.warning(f"ACC_STATUS decode error: {e}")
            return

        with self._lock:
            # GET_ACC_STATE 는 enum (NamedSignalValue) — int 로 강제 변환
            self._acc_info.status         = int(signals["GET_ACC_STATE"])
            self._acc_info.set_speed      = int(signals["GET_ACC_SPD"])
            self._acc_info.distance_level = int(signals["GET_ACC_LVL"])

    def _parse_ecu_heartbeat(self, data: bytes) -> None:
        """0x410 ECU_HEARTBEAT 파싱 (cantools decode)."""
        try:
            signals = _msg("ECU_HEARTBEAT").decode(data)
        except Exception as e:
            logger.warning(f"ECU_HEARTBEAT decode error: {e}")
            return
        with self._lock:
            self._hb_last_rx = time.time()
            self._ecu_status.error_code = int(signals["ERR_ECU"])
            # heartbeat_ok 는 _hb_watchdog 가 갱신

    # =========================================================================
    # 내부: Heartbeat 감시
    # =========================================================================

    def _hb_watchdog(self) -> None:
        """50ms 간격으로 마지막 HB 수신 시각 확인 → heartbeat_ok 갱신."""
        while self._running:
            time.sleep(0.05)
            now = time.time()
            with self._lock:
                ok = (now - self._hb_last_rx) < HB_TIMEOUT_SEC
                self._ecu_status.heartbeat_ok = ok
            if not ok:
                logger.warning("ECU Heartbeat timeout!")

    # =========================================================================
    # 내부: CAN 송신 헬퍼
    # =========================================================================

    def _send_raw(self, msg_id: int, data: bytes) -> None:
        """CAN 메시지 송신. 버스 미연결 시 시뮬레이션 로그만 출력."""
        if self._bus is None:
            logger.debug(f"[SIM] TX 0x{msg_id:03X}: {data.hex()}")
            return
        try:
            msg = can.Message(
                arbitration_id=msg_id,
                data=data,
                is_extended_id=False,  # 표준 CAN 11-bit
            )
            self._bus.send(msg)
        except Exception as e:
            logger.error(f"CAN TX error (0x{msg_id:03X}): {e}")
