__version__ = "0.0.1"
__author__ = "Wis3754"

"""
ACC Project - Raspberry Pi CAN Interface Module
================================================
담당: Raspi CAN 모듈
역할:
  - HMI/Fusion 모듈로부터 데이터를 받아 ECU로 CAN 송신
  - ECU로부터 CAN 수신하여 HMI 모듈에 제공 (폴링)
  - Heartbeat 감시 및 ERR 처리

CAN 메시지 주기/타임아웃 (SWR 기준):
  TX (Raspi → ECU):
    0x100 RASPI_ACC_CMD   : 버튼/페달/ACC설정  20ms  (SWR020: 버튼 CAN 전송)
    0x110 RASPI_SENSOR    : Fusion 데이터      20ms  (SWR013: 수신 주기 20ms)
  RX (ECU → Raspi):
    0x400 ECU_ACC_STATUS  : ACC 상태 피드백    50ms  (SWR018: GUI 갱신 50ms)
    0x410 ECU_HEARTBEAT   : ECU HB/ERR        —     (타임아웃 150ms = 3주기)

타임아웃:
  RASPI_SENSOR 미갱신 60ms → ERR_SENSOR 설정  (SWR013: 타임아웃 60ms)
  ECU_HEARTBEAT 미수신 150ms → heartbeat_ok=False

관련 요구사항: SYS030, SYS031, SWR013, SWR018, SWR019, SWR020, SWR030
"""

import threading # 여러개 동시에 실행 (TX, RX, HB)
import time
import logging
from typing import Optional

try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False

logger = logging.getLogger(__name__)

# ── CAN Message IDs (DBC 기준) ──────────────────────────────────────────────
MSG_ID_RASPI_ACC_CMD    = 0x100   # Raspi → ECU: 버튼/페달/설정
MSG_ID_RASPI_SENSOR     = 0x110   # Raspi → ECU: Fusion 데이터
MSG_ID_ECU_ACC_STATUS   = 0x400   # ECU → Raspi: ACC 상태 -> 왜 여기에 raspi HB/ERR이 포함되어있지?
MSG_ID_ECU_HEARTBEAT    = 0x410   # ECU → Raspi: Heartbeat/ERR

# ── TX 주기 (ms) ─────────────────────────────────────────────────────────────
TX_PERIOD_MS            = 20      # RASPI_ACC_CMD, RASPI_SENSOR 송신 주기
GUI_REFRESH_MS          = 50      # HMI 폴링 주기 (참고용)

# ── Heartbeat 타임아웃 ────────────────────────────────────────────────────────
HB_TIMEOUT_SEC          = 0.15    # 150ms: 3주기(50ms) 미수신 시 False

# ── ACC 상태 enum ─────────────────────────────────────────────────────────────
ACC_STATUS_OFF          = 0
ACC_STATUS_STANDBY      = 1
ACC_STATUS_CRUISING     = 2
ACC_STATUS_FOLLOWING    = 3
ACC_STATUS_FAULT        = 4


class CanInterface:
    """
    Raspberry Pi CAN 통신 모듈.

    사용법:
        can_mod = CanInterface(channel='can0', bustype='socketcan')
        can_mod.start()
        ...
        can_mod.send_button_input({"btn_acc_off": True, ...})
        state = can_mod.get_acc_state()
        ...
        can_mod.stop()
    """

    def __init__(self, channel: str = 'can0', bustype: str = 'socketcan'):
        self._channel = channel
        self._bustype = bustype
        self._bus: Optional[object] = None # 연결 전
        self._running = False # 스레드 플래그

        # ── TX 버퍼 ──────────────────────────────────────────────────────────
        self._lock = threading.Lock() # 동시 같은 버퍼(변수) 충돌 방지 (RX/TX/HMI)

        # button_input: Edge Trigger — TX 후 자동 클리어
        self._button_input = {
            "btn_acc_off":    False,
            "btn_acc_set":    False,
            "btn_acc_res":    False,
            "btn_acc_cancel": False,
        }

        # pedal_input: 연속값 — 최신값 유지
        self._pedal_input = {
            "brake":     False,
            "accel_pwm": 0,        # 0~100 (%)
        }

        # acc_setting: 최신값 유지
        self._acc_setting = {
            "set_speed":      0,   # cm/s (절대값)
            "distance_level": 3,   # 1/2/3
        }

        # fusion_data: Fusion 모듈이 갱신
        self._fusion_data = {
            "detected": False,
            "distance": 0,         # cm (uint8)
        }
        self._fusion_last_update = 0.0  # 마지막 갱신 시각

        # ── RX 버퍼 ──────────────────────────────────────────────────────────
        self._acc_state = {
            "status":         ACC_STATUS_OFF,
            "set_speed":      0,
            "distance_level": 3,
        }

        self._vehicle_info = {
            "current_speed": 0.0,  # cm/s
        }

        self._ecu_status = {
            "heartbeat_ok": False, # 연결 전
            "error_code":   0,
        }

        self._hb_last_rx = 0.0     # 마지막 HB 수신 시각

        # ── 스레드 ────────────────────────────────────────────────────────────
        self._tx_thread: Optional[threading.Thread] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._hb_thread: Optional[threading.Thread] = None

    # =========================================================================
    # 생명주기
    # =========================================================================

    def start(self) -> None:
        """CAN 버스 초기화 + TX/RX/HB 스레드 시작."""
        if self._running: # 이미 실행 중
            logger.warning("CanInterface already running.")
            return

        if CAN_AVAILABLE:
            try: # CAN 연결 시도
                self._bus = can.interface.Bus(
                    channel=self._channel,
                    bustype=self._bustype
                )
                logger.info(f"CAN bus opened: {self._channel} ({self._bustype})")
            except Exception as e: # CAN 연결 실패
                logger.error(f"CAN bus open failed: {e}. Running in simulation mode.")
                self._bus = None
        else: # CAN 라이브러리 미설치
            logger.warning("python-can not installed. Running in simulation mode.")
            self._bus = None

        self._running = True
        self._hb_last_rx = time.time()  # 초기화 시 타임아웃 방지

        # target: 실행할 함수, daemon: 프로그램 종료시 종료 여부, name: 스레드 이름 (logging에 표시)
        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True, name="CAN_TX")
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True, name="CAN_RX")
        self._hb_thread = threading.Thread(target=self._hb_watchdog, daemon=True, name="CAN_HB")

        # 실행
        self._tx_thread.start()
        self._rx_thread.start()
        self._hb_thread.start()
        logger.info("CanInterface started.")

    def stop(self) -> None:
        """TX 타이머/RX 스레드 종료 + CAN 버스 해제."""
        self._running = False # 스레드 종료 (while 루프 종료 플래그)

        for t in (self._tx_thread, self._rx_thread, self._hb_thread):
            if t and t.is_alive():
                t.join(timeout=1.0)

        if self._bus:
            try:
                self._bus.shutdown()
            except Exception as e: # CAN 버스 해제 실패
                logger.error(f"CAN bus shutdown error: {e}")
            self._bus = None

        logger.info("CanInterface stopped.")

    def is_connected(self) -> bool:
        """CAN 버스 연결 상태 반환. 시뮬레이션 모드면 False."""
        return self._bus is not None # Ture or False 반환

    # =========================================================================
    # HMI → CAN 모듈 (송신용 setter)
    # =========================================================================

    def send_button_input(self, button_input: dict) -> None:
        """
        버튼 Edge Trigger 입력을 CAN 모듈에 전달.
        호출 시점: 버튼 클릭 즉시 (이벤트 드리븐)
        다음 20ms TX 주기에 CAN 전송 후 자동 클리어.

        Args:
            button_input: {
                "btn_acc_off":    bool,
                "btn_acc_set":    bool,
                "btn_acc_res":    bool,
                "btn_acc_cancel": bool,
            }
        """
        with self._lock:
            for key in self._button_input: # 버튼 4개 (ACC OFF/SET/RES/CANCEL)
                if key in button_input: # HMI에서 전달된 버튼만 업데이트
                    self._button_input[key] = self._button_input[key] or bool(button_input[key]) # 여러번 눌려도 한번 전송

    def send_pedal_input(self, pedal_input: dict) -> None:
        """
        페달 연속값을 CAN 모듈에 전달.
        호출 시점: 페달값 변경 즉시 (디바운싱 없음)

        Args:
            pedal_input: {
                "brake":     bool,   # 0% 초과 시 True
                "accel_pwm": int,    # 0~100 (%)
            }
        """
        with self._lock:
            if "brake" in pedal_input: # pedal_input에 brake이 존재하는지 (안전장치 느낌)
                self._pedal_input["brake"] = bool(pedal_input["brake"]) # 브레이크도 버튼처럼 여러번 눌러도 한번 전송?
            if "accel_pwm" in pedal_input: # pedal_input에 accel_pwm이 존재하는지
                self._pedal_input["accel_pwm"] = max(0, min(127, int(pedal_input["accel_pwm"])))

    def send_acc_setting(self, acc_setting: dict) -> None:
        """
        ACC 설정 변경값을 CAN 모듈에 전달.
        HMI에서 현재값 + 증감분을 계산한 절대값을 전달.

        Args:
            acc_setting: {
                "set_speed":      int,   # cm/s 절대값
                "distance_level": int,   # 1/2/3
            }
        """
        with self._lock:
            if "set_speed" in acc_setting:
                spd = int(acc_setting["set_speed"])
                self._acc_setting["set_speed"] = max(0, min(127, spd))
            if "distance_level" in acc_setting:
                lvl = int(acc_setting["distance_level"])
                self._acc_setting["distance_level"] = max(1, min(3, lvl))

    # =========================================================================
    # Fusion → CAN 모듈
    # =========================================================================

    def update_fusion_data(self, fusion_data: dict) -> None:
        """
        Fusion 모듈이 산출한 전방 차량 데이터를 CAN 모듈에 전달.
        호출 시점: Fusion 모듈의 센서 처리 완료 시 (비동기)

        Args:
            fusion_data: {
                "detected": bool,
                "distance": int,   # cm
            }
        """
        with self._lock:
            if "detected" in fusion_data:
                self._fusion_data["detected"] = bool(fusion_data["detected"])
            if "distance" in fusion_data:
                self._fusion_data["distance"] = max(0, min(255, int(fusion_data["distance"]))) # VEH_DIST uint8
            self._fusion_last_update = time.time()

    # =========================================================================
    # CAN 모듈 → HMI (수신 데이터 getter)
    # =========================================================================

    def get_acc_state(self) -> dict:
        """
        ECU에서 수신한 ACC 상태 정보 반환.
        HMI의 50ms GUI 갱신 타이머에서 폴링.

        Returns:
            {
                "status":         int,   # 0=OFF,1=STANDBY,2=CRUISING,3=FOLLOWING,4=FAULT
                "set_speed":      int,   # cm/s
                "distance_level": int,   # 1/2/3
            }
        """
        with self._lock:
            return dict(self._acc_state)

    def get_vehicle_info(self) -> dict:
        """
        ECU에서 수신한 자차 속도 정보 반환.

        Returns:
            {
                "current_speed": float,   # cm/s
            }
        """
        with self._lock:
            return dict(self._vehicle_info)

    def get_ecu_status(self) -> dict:
        """
        ECU 통신 상태 및 에러 정보 반환.

        Returns:
            {
                "heartbeat_ok": bool,   # 150ms 이내 HB 수신 여부
                "error_code":   int,    # 0=정상
            }
        """
        with self._lock:
            return dict(self._ecu_status)

    # =========================================================================
    # 내부: TX 루프 (20ms 주기)
    # =========================================================================

    def _tx_loop(self) -> None:
        """20ms 주기로 RASPI_ACC_CMD, RASPI_SENSOR 송신."""
        period = TX_PERIOD_MS / 1000.0 # s -> ms
        while self._running:
            t_start = time.monotonic() # 루프 시작 시각 (정확한 주기 유지용)

            self._tx_acc_cmd()
            self._tx_sensor()

            elapsed = time.monotonic() - t_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time) # 주기 맞추기 용으로 남은 시간만큼 슬립

    def _tx_acc_cmd(self) -> None:
        """0x100 RASPI_ACC_CMD 송신."""
        with self._lock:
            btn_off    = int(self._button_input["btn_acc_off"])
            btn_set    = int(self._button_input["btn_acc_set"])
            btn_res    = int(self._button_input["btn_acc_res"])
            btn_cancel = int(self._button_input["btn_acc_cancel"])
            brake      = int(self._pedal_input["brake"])
            accel_pwm  = self._pedal_input["accel_pwm"]
            set_spd    = self._acc_setting["set_speed"]
            set_lvl    = self._acc_setting["distance_level"]

            # Edge trigger: TX 후 클리어
            self._button_input["btn_acc_off"]    = False
            self._button_input["btn_acc_set"]    = False
            self._button_input["btn_acc_res"]    = False
            self._button_input["btn_acc_cancel"] = False

        # ── 패킹 (DBC 0x100, 8 bytes) ────────────────────────────────────────
        # Byte 0: SET_ACCEL_PWM [0|8] signed(-128 offset) → 0~100 그대로 전송
        # Byte 1: BTN_BRAKE[0], BTN_ACC_OFF[1], BTN_ACC_SET[2],
        #         BTN_ACC_RES[3], BTN_ACC_CANCEL[4]
        # Byte 2: SET_ACC_SPD [16|8] signed
        # Byte 3: SET_ACC_LVL [24|8]
        # Byte 4~7: 예약 (0x00)
        btn_byte = (
            (brake      & 0x1) << 0 |
            (btn_off    & 0x1) << 1 |
            (btn_set    & 0x1) << 2 |
            (btn_res    & 0x1) << 3 |
            (btn_cancel & 0x1) << 4
        )
        data = bytes([
            accel_pwm & 0xFF,
            btn_byte  & 0xFF,
            set_spd   & 0xFF,
            set_lvl   & 0xFF,
            0x00, 0x00, 0x00, 0x00, # 뒤 4바이트는 예약 (0x00) (CAN 메세지가 최대 8바이트라서)
        ])
        self._send_raw(MSG_ID_RASPI_ACC_CMD, data)

    def _tx_sensor(self) -> None:
        """0x110 RASPI_SENSOR 송신. Fusion 100ms 미갱신 시 ERR_SENSOR 설정."""
        with self._lock:
            detected = int(self._fusion_data["detected"])
            distance = self._fusion_data["distance"]
            last_upd = self._fusion_last_update

        now = time.time()
        err_sensor = 0x01 if (now - last_upd > 0.06 and last_upd != 0.0) else 0x00  # SWR013: 60ms 타임아웃

        # HB_SENSOR: 0~255 카운터 (매 TX마다 +1 wrap)
        if not hasattr(self, '_hb_sensor_counter'):
            self._hb_sensor_counter = 0
        self._hb_sensor_counter = (self._hb_sensor_counter + 1) % 256

        # ── 패킹 (DBC 0x110, 4 bytes) ────────────────────────────────────────
        # Byte 0: VEH_DET [0|1]
        # Byte 1: VEH_DIST [8|8] cm uint8
        # Byte 2: HB_SENSOR [16|8]
        # Byte 3: ERR_SENSOR [24|8]
        data = bytes([
            detected              & 0x01,
            distance              & 0xFF,
            self._hb_sensor_counter,
            err_sensor            & 0xFF,
        ])
        self._send_raw(MSG_ID_RASPI_SENSOR, data)

    # =========================================================================
    # 내부: RX 루프
    # =========================================================================

    def _rx_loop(self) -> None:
        """CAN 수신 스레드. ECU 메시지를 파싱하여 내부 버퍼 갱신."""
        while self._running:
            if self._bus is None: # CAN 버스 미연결 또는 시뮬레이션 모드
                time.sleep(0.05)
                continue
            try:
                msg = self._bus.recv(timeout=0.1)
                if msg is None:
                    continue
                if msg.arbitration_id == MSG_ID_ECU_ACC_STATUS: # 메세지 구분, 나머지는 Raspi 송신 메세지 아니어서 무시
                    self._parse_acc_status(msg.data)
                elif msg.arbitration_id == MSG_ID_ECU_HEARTBEAT:
                    self._parse_ecu_heartbeat(msg.data)
            except Exception as e:
                logger.error(f"CAN RX error: {e}") 

    def _parse_acc_status(self, data: bytes) -> None:
        """
        0x400 ECU_ACC_STATUS 파싱.
        Byte 0-1: GET_SPD_AVG  (uint16, factor 0.01) → cm/s float
        Byte 2:   GET_ACC_STATUS (uint8, 0~4)
        Byte 3:   GET_ACC_SPD   (int8)
        Byte 4:   GET_ACC_LVL   (uint8, 1~3)
        """
        if len(data) < 5: # 인덱스 오류 방지용 (짧은 프레임 수신 시 로그만 남기고 무시)
            logger.warning(f"ECU_ACC_STATUS: short frame ({len(data)} bytes)")
            return

        spd_raw    = (data[1] << 8) | data[0]          # little-endian uint16
        spd_avg    = spd_raw * 0.01                     # cm/s로 변환

        acc_status = data[2] & 0x07
        acc_spd    = data[3] if data[3] < 128 else data[3] - 256  # int8
        acc_lvl    = max(1, min(3, data[4]))

        with self._lock:
            self._acc_state["status"]         = acc_status
            self._acc_state["set_speed"]      = acc_spd
            self._acc_state["distance_level"] = acc_lvl
            self._vehicle_info["current_speed"] = round(spd_avg, 2)

    def _parse_ecu_heartbeat(self, data: bytes) -> None:
        """
        0x410 ECU_HEARTBEAT 파싱.
        Byte 0: HB_ECU  (uint8 카운터)
        Byte 1: ERR_ECU (uint8, 0=정상)
        """
        if len(data) < 2: # 인덱스 오류 방지용 (짧은 프레임 수신 시 로그만 남기고 무시)
            logger.warning(f"ECU_HEARTBEAT: short frame ({len(data)} bytes)")
            return

        err_ecu = data[1]

        with self._lock:
            self._hb_last_rx               = time.time()
            self._ecu_status["error_code"] = err_ecu
            # heartbeat_ok는 _hb_watchdog에서 갱신

    # =========================================================================
    # 내부: Heartbeat 감시 (150ms 타임아웃)
    # =========================================================================

    def _hb_watchdog(self) -> None:
        """150ms 이내 ECU HB 미수신 시 heartbeat_ok = False."""
        while self._running:
            time.sleep(0.05) # 50ms마다 확인
            now = time.time()
            with self._lock:
                ok = (now - self._hb_last_rx) < HB_TIMEOUT_SEC
                self._ecu_status["heartbeat_ok"] = ok
            if not ok:
                logger.warning("ECU Heartbeat timeout!")

    # =========================================================================
    # 내부: CAN 송신 헬퍼
    # =========================================================================

    def _send_raw(self, msg_id: int, data: bytes) -> None:
        """CAN 메시지 송신. 버스 미연결 시 시뮬레이션 로그만 출력."""
        if self._bus is None: # CAN 버스 미연결 또는 시뮬레이션 모드
            logger.debug(f"[SIM] TX 0x{msg_id:03X}: {data.hex()}")
            return
        try:
            msg = can.Message(
                arbitration_id=msg_id,
                data=data,
                is_extended_id=False # 표준 CAN ID (11-bit)
            )
            self._bus.send(msg) # 실제 전송
        except Exception as e:
            logger.error(f"CAN TX error (0x{msg_id:03X}): {e}") # 송신 실패해도 프로그램 멈추지 않도록
