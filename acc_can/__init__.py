"""
ACC Project - Raspberry Pi CAN Interface Module
================================================
담당: Raspi CAN 모듈 (DBC-driven, cantools 기반)

역할:
  - HMI/Fusion 모듈로부터 데이터를 받아 ECU로 CAN 송신
  - ECU로부터 CAN 수신하여 HMI 모듈에 제공 (폴링)
  - Heartbeat 감시 및 ERR 처리

구조 (2026-04-23 리팩터):
  _dbc.py    — DBC 로드 + ID 상수 + msg()/msg_id() 헬퍼
  _codec.py  — encode/decode pure function (메시지별)
  __init__.py — CanInterface 클래스 (스레딩 + 상태 + 공개 API)

CAN 메시지 (DBC 2026-04-23 기준):
  TX (SENSOR → ECU):
    0x110 SENSOR_FUSION     전방차량 감지/거리    20ms
    0x111 SENSOR_HEARTBEAT  RPi HB/ERR            20ms
    0x120 VEH_CTRL          brake + accel_pwm     20ms  (SYS006/007, SAF004/015)
    0x510 ACC_CTRL          ACC 버튼 + 목표       50ms  (SYS030)
  RX (ECU → SENSOR / MTR → SENSOR):
    0x300 MTR_SPD_FB        자차 평균속도          10ms  (GET_SPD_AVG int16)
    0x410 ECU_HEARTBEAT     ECU HB/ERR            10ms  (SAF018 ASIL-B)
    0x520 ACC_STATUS        ACC 상태 피드백       50ms  (SYS031)

타임아웃:
  SENSOR_FUSION 미갱신 60ms → ERR_SENSOR 설정  (SWR013)
  ECU_HEARTBEAT 미수신 150ms → heartbeat_ok=False

관련 요구사항: SYS006, SYS007, SYS015, SYS025, SYS030, SYS031,
              SWR013, SWR018, SWR019, SWR020, SWR030,
              SAF004, SAF015, SAF018
"""

__version__ = "0.3.0"
__author__ = "Wis3754"

import logging
import threading
import time
from dataclasses import replace
from typing import Optional

from interfaces.acc_info import AccInfo
from interfaces.acc_setting import AccSetting
from interfaces.acc_status import AccStatus
from interfaces.button_input import ButtonInput
from interfaces.ecu_status import EcuStatus
from interfaces.fusion_data import FusionData
from interfaces.pedal_input import PedalInput
from interfaces.vehicle_info import VehicleInfo

from . import _codec
from ._dbc import (
    MSG_ID_ACC_CTRL,
    MSG_ID_ACC_STATUS,
    MSG_ID_ECU_HEARTBEAT,
    MSG_ID_MTR_SPD_FB,
    MSG_ID_SENSOR_FUSION,
    MSG_ID_SENSOR_HEARTBEAT,
    MSG_ID_VEH_CTRL,
)

try:
    import can

    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False

logger = logging.getLogger(__name__)

# ── TX 주기 ──────────────────────────────────────────────────────────────────
TX_PERIOD_20MS_SEC = 0.020  # VEH_CTRL, SENSOR_FUSION, SENSOR_HEARTBEAT
TX_PERIOD_50MS_SEC = 0.050  # ACC_CTRL
TX_POLL_SEC        = 0.005  # TX 루프 폴링 간격
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
        self._bus: Optional[object] = None
        self._running = False

        # ── 동기화 ───────────────────────────────────────────────────────────
        self._lock = threading.Lock()  # RX/TX/HMI 간 상태 버퍼 보호

        # ── TX 상태 버퍼 ─────────────────────────────────────────────────────
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

        # ── RX 상태 버퍼 ─────────────────────────────────────────────────────
        self._acc_info = AccInfo(
            status=AccStatus.OFF.value,
            set_speed=0,
            distance_level=3,
        )

        # 자차 평균속도 (GET_SPD_AVG, int16 × 0.02 cm/s).
        # _parse_mtr_spd_fb 가 0x300 MTR_SPD_FB 10ms 수신 시 갱신.
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
        """TX/RX/HB 스레드 종료 + CAN 버스 해제."""
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
            return replace(self._acc_info)

    def get_vehicle_info(self) -> VehicleInfo:
        """자차 평균속도 반환 (MTR_SPD_FB 10ms 로 갱신)."""
        with self._lock:
            return replace(self._vehicle_info)

    def get_ecu_status(self) -> EcuStatus:
        """ECU 통신 상태 + 에러 코드 반환."""
        with self._lock:
            return replace(self._ecu_status)

    # =========================================================================
    # 내부: TX 루프 (multi-rate: 20ms + 50ms)
    # =========================================================================

    def _tx_loop(self) -> None:
        """
        20ms 주기: VEH_CTRL, SENSOR_FUSION, SENSOR_HEARTBEAT
        50ms 주기: ACC_CTRL
        5ms 폴링으로 두 주기 독립 관리.
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
            pedal = replace(self._pedal_input)  # snapshot
        try:
            data = _codec.encode_veh_ctrl(pedal)
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
            buttons = replace(self._button_input)  # snapshot
            setting = replace(self._acc_setting)
            # Edge trigger: TX 후 클리어
            self._button_input.btn_acc_off = False
            self._button_input.btn_acc_set = False
            self._button_input.btn_acc_res = False
            self._button_input.btn_acc_cancel = False
        try:
            data = _codec.encode_acc_ctrl(buttons, setting)
        except Exception as e:
            logger.error(f"ACC_CTRL encode error: {e}")
            return
        self._send_raw(MSG_ID_ACC_CTRL, data)

    def _tx_sensor_fusion(self) -> None:
        """0x110 SENSOR_FUSION (20ms) — 전방 차량 감지 + 거리(mm)."""
        with self._lock:
            fusion = replace(self._fusion_data)  # snapshot
        try:
            data = _codec.encode_sensor_fusion(fusion)
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

        try:
            data = _codec.encode_sensor_heartbeat(self._hb_sensor_counter, err_sensor)
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
                elif msg.arbitration_id == MSG_ID_MTR_SPD_FB:
                    self._parse_mtr_spd_fb(msg.data)
            except Exception as e:
                logger.error(f"CAN RX error: {e}")

    def _parse_acc_status(self, data: bytes) -> None:
        """0x520 ACC_STATUS → self._acc_info."""
        try:
            acc_info = _codec.decode_acc_status(data)
        except Exception as e:
            logger.warning(f"ACC_STATUS decode error: {e}")
            return
        with self._lock:
            self._acc_info.status         = acc_info.status
            self._acc_info.set_speed      = acc_info.set_speed
            self._acc_info.distance_level = acc_info.distance_level

    def _parse_ecu_heartbeat(self, data: bytes) -> None:
        """0x410 ECU_HEARTBEAT → self._ecu_status.error_code + HB 수신 시각 갱신."""
        try:
            err_ecu = _codec.decode_ecu_heartbeat(data)
        except Exception as e:
            logger.warning(f"ECU_HEARTBEAT decode error: {e}")
            return
        with self._lock:
            self._hb_last_rx = time.time()
            self._ecu_status.error_code = err_ecu
            # heartbeat_ok 는 _hb_watchdog 가 갱신

    def _parse_mtr_spd_fb(self, data: bytes) -> None:
        """0x300 MTR_SPD_FB → self._vehicle_info.current_speed (cm/s)."""
        try:
            speed_cms = _codec.decode_mtr_spd_fb(data)
        except Exception as e:
            logger.warning(f"MTR_SPD_FB decode error: {e}")
            return
        with self._lock:
            self._vehicle_info.current_speed = round(speed_cms, 2)

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
