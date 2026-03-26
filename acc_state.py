"""
ACC State Machine & Data Model
Ref: STK009, SYS001-SYS005, SYS029, SWR001
"""
from enum import Enum, auto
from dataclasses import dataclass


class AccState(Enum):
    OFF = auto()
    STANDBY = auto()
    CRUISING = auto()
    FOLLOWING = auto()
    FAULT = auto()


# 활성 상태 집합 (자주 사용되므로 상수화)
_ACTIVE = frozenset({AccState.CRUISING, AccState.FOLLOWING})

# Button availability per state (SYS029 / STK021)
BUTTON_AVAILABILITY: dict[AccState, dict[str, bool]] = {
    AccState.OFF: {
        "acc_toggle": True,
        "set": False, "res": False,
        "speed_up": False, "speed_down": False,
        "dist_1": False, "dist_2": False, "dist_3": False,
        "pause": False,
    },
    AccState.STANDBY: {
        "acc_toggle": True,
        "set": True, "res": True,
        "speed_up": False, "speed_down": False,
        "dist_1": True, "dist_2": True, "dist_3": True,
        "pause": False,
    },
    AccState.CRUISING: {
        "acc_toggle": True,
        "set": False, "res": False,
        "speed_up": True, "speed_down": True,
        "dist_1": True, "dist_2": True, "dist_3": True,
        "pause": True,
    },
    AccState.FOLLOWING: {
        "acc_toggle": True,
        "set": False, "res": False,
        "speed_up": True, "speed_down": True,
        "dist_1": True, "dist_2": True, "dist_3": True,
        "pause": True,
    },
    AccState.FAULT: {
        "acc_toggle": True,
        "set": False, "res": False,
        "speed_up": False, "speed_down": False,
        "dist_1": False, "dist_2": False, "dist_3": False,
        "pause": False,
    },
}

# 1/10 스케일카 파라미터 (SYS032 / STK024)
MIN_SET_SPEED_MPS = 0.3   # m/s
SPEED_INCREMENT_MPS = 0.1  # m/s per click
DEFAULT_DISTANCE_LEVEL = 3  # ACC 활성화 시 기본 안전거리 단계


@dataclass
class AccData:
    """Runtime data exchanged between HMI and ECU (SYS030, SYS031)."""
    state: AccState = AccState.OFF
    current_speed_mps: float = 0.0
    set_speed_mps: float = 0.0
    prev_set_speed_mps: float = 0.0
    distance_level: int = DEFAULT_DISTANCE_LEVEL
    front_vehicle_detected: bool = False
    front_distance_mm: int = 0
    front_rel_speed_mps: float = 0.0  # 음수=접근, 양수=이탈
    fault_warning: bool = False
    brake_pressed: bool = False
    accel_pressed: bool = False
    brake_pct: float = 0.0
    accel_pct: float = 0.0


class AccController:
    """
    Simulated ACC state machine for standalone HMI testing.
    In production, state transitions happen on Main ECU (MPC5606B).
    Ref: SWR001, SWR002, SWR003, SWR026-SWR029, SWR031
    """

    def __init__(self):
        self.data = AccData()

    def toggle_acc(self):
        """ACC ON/OFF 토글 (STK022, SYS001, SYS005)."""
        s = self.data.state
        if s == AccState.OFF:
            self._try_activate()
        elif s == AccState.FAULT:
            self.data.state = AccState.OFF
            self.data.fault_warning = False
        else:
            self.data.state = AccState.OFF
            self.data.set_speed_mps = 0.0

    def press_set(self):
        """SET – STANDBY에서만 유효 (SYS027, SWR026)."""
        if self.data.state != AccState.STANDBY:
            return
        speed = max(self.data.current_speed_mps, MIN_SET_SPEED_MPS)
        self.data.set_speed_mps = speed
        self.data.prev_set_speed_mps = speed
        self._enter_active()

    def press_res(self):
        """RES – STANDBY에서만 유효 (SYS027, SWR026)."""
        if self.data.state != AccState.STANDBY:
            return
        self.data.set_speed_mps = self.data.prev_set_speed_mps
        self._enter_active()

    def press_speed_up(self):
        """속도+ (SYS003, SYS004)."""
        if self.data.state in _ACTIVE:
            self.data.set_speed_mps += SPEED_INCREMENT_MPS
            self.data.prev_set_speed_mps = self.data.set_speed_mps

    def press_speed_down(self):
        """속도- (SYS003, SYS004, SWR031)."""
        if self.data.state in _ACTIVE:
            self.data.set_speed_mps = max(
                self.data.set_speed_mps - SPEED_INCREMENT_MPS, MIN_SET_SPEED_MPS
            )
            self.data.prev_set_speed_mps = self.data.set_speed_mps

    def set_distance_level(self, level: int):
        """거리설정 1/2/3 (SYS008-SYS010)."""
        if self.data.state in (AccState.STANDBY, *_ACTIVE):
            self.data.distance_level = max(1, min(3, level))

    def press_pause(self):
        """일시정지 → STANDBY (SYS028, SWR027)."""
        if self.data.state in _ACTIVE:
            self._save_and_standby()

    def set_brake(self, pct: float):
        """브레이크 페달 (0~100%). >0이면 오버라이드 (SYS006, SWR002)."""
        pct = max(0.0, min(100.0, pct))
        was_pressed = self.data.brake_pressed
        self.data.brake_pct = pct
        self.data.brake_pressed = pct > 0
        if not was_pressed and self.data.brake_pressed and self.data.state in _ACTIVE:
            self._save_and_standby()

    def set_accel(self, pct: float):
        """액셀 페달 (0~100%). >0이면 오버라이드 (SYS007, SWR003)."""
        pct = max(0.0, min(100.0, pct))
        was_pressed = self.data.accel_pressed
        self.data.accel_pct = pct
        self.data.accel_pressed = pct > 0
        if not was_pressed and self.data.accel_pressed and self.data.state in _ACTIVE:
            self._save_and_standby()

    def trigger_fault(self):
        """고장 감지 → FAULT (SYS005, SWR016)."""
        if self.data.state != AccState.OFF:
            self.data.state = AccState.FAULT
            self.data.fault_warning = True

    def set_front_vehicle(self, detected: bool, distance_mm: int = 0):
        """전방 차량 갱신 – CRUISING↔FOLLOWING 전이 (SYS003, SYS004)."""
        self.data.front_vehicle_detected = detected
        self.data.front_distance_mm = distance_mm
        if self.data.state == AccState.CRUISING and detected:
            self.data.state = AccState.FOLLOWING
        elif self.data.state == AccState.FOLLOWING and not detected:
            self.data.state = AccState.CRUISING

    def get_button_availability(self) -> dict[str, bool]:
        return BUTTON_AVAILABILITY[self.data.state]

    # ── Private helpers ──

    def _try_activate(self):
        """OFF → CRUISING/FOLLOWING (SYS001, SYS033, SWR029)."""
        if self.data.brake_pressed or self.data.accel_pressed:
            return
        speed = max(self.data.current_speed_mps, MIN_SET_SPEED_MPS)
        self.data.set_speed_mps = speed
        self.data.prev_set_speed_mps = speed
        self.data.distance_level = DEFAULT_DISTANCE_LEVEL
        self._enter_active()

    def _enter_active(self):
        self.data.state = (
            AccState.FOLLOWING if self.data.front_vehicle_detected else AccState.CRUISING
        )

    def _save_and_standby(self):
        self.data.prev_set_speed_mps = self.data.set_speed_mps
        self.data.state = AccState.STANDBY
