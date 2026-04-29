"""
ACC 상태 → 화면 표시 매핑.

상태·속도·거리단계는 `acc_can` 모듈(CanInterface)이 소유한다. HMI 는 50ms 주기로
`get_acc_info() / get_vehicle_info() / get_ecu_status()` 를 폴링해 렌더링만 수행한다.
이 모듈은 그 폴링 결과(`AccStatus`) 를 화면 표시용 값(라벨, 색, 활성/비활성 플래그,
파생 텍스트) 으로 번역하는 **상태 해석 레이어**다.

대원칙: `AccStatus` 를 보고 무언가를 결정하는 코드는 모두 여기에 모은다. `hmi_widgets`
는 이 모듈에 의존하지 않으며 `hmi_gui` 의 `HmiWindow._refresh` 가 이 모듈의 헬퍼를
호출해 위젯에 넣을 원시값(숫자/색/문자열) 을 만든다.

Ref: STK020, STK021, SYS019, SYS029, SYS032, SYS034, SWR018, SWR028, SWR031
"""
from interfaces.acc_status import AccStatus


# ── 상태 분류 ───────────────────────────────────────────────────

# 활성 상태 집합 (CRUISING/FOLLOWING)
ACTIVE_STATUSES = frozenset({AccStatus.CRUISING, AccStatus.FOLLOWING})

# 운전자 오버라이드 표시 대상 상태 (STANDBY 포함 — pedal 인가만으로 OVERRIDE 표시)
_OVERRIDE_VISIBLE_STATUSES = frozenset({
    AccStatus.STANDBY, AccStatus.CRUISING, AccStatus.FOLLOWING,
})


# ── 버튼 가용성 (SYS029 / STK021) ─────────────────────────────────
BUTTON_AVAILABILITY: dict[AccStatus, dict[str, bool]] = {
    AccStatus.OFF: {
        "acc_toggle": True,
        "set": False, "res": False,
        "speed_up": False, "speed_down": False,
        "dist_1": False, "dist_2": False, "dist_3": False,
        "pause": False,
    },
    AccStatus.STANDBY: {
        "acc_toggle": True,
        "set": True, "res": True,
        "speed_up": False, "speed_down": False,
        "dist_1": True, "dist_2": True, "dist_3": True,
        "pause": False,
    },
    AccStatus.CRUISING: {
        "acc_toggle": True,
        "set": False, "res": False,
        "speed_up": True, "speed_down": True,
        "dist_1": True, "dist_2": True, "dist_3": True,
        "pause": True,
    },
    AccStatus.FOLLOWING: {
        "acc_toggle": True,
        "set": False, "res": False,
        "speed_up": True, "speed_down": True,
        "dist_1": True, "dist_2": True, "dist_3": True,
        "pause": True,
    },
    AccStatus.FAULT: {
        "acc_toggle": True,
        "set": False, "res": False,
        "speed_up": False, "speed_down": False,
        "dist_1": False, "dist_2": False, "dist_3": False,
        "pause": False,
    },
}


# ── 속도/거리 파라미터 (SYS032 / STK024 / SWR031) ────────────────
# 1/10 스케일카, 단위 cm/s
MIN_SET_SPEED_CMS = 5         # 최소 설정 속도 (SWR031)
MAX_SET_SPEED_CMS = 500       # DBC SET_ACC_SPD int16 상한 (-500~500 cm/s)
SPEED_INCREMENT_CMS = 1       # SPD+/- 클릭당 증감량
SPEED_GAUGE_MAX_CMS = 500     # 속도 게이지 최대 눈금 (DBC GET_ACC_SPD int16 상한 일치)
DEFAULT_DISTANCE_LEVEL = 3    # 기본 거리 단계 (SYS034)


# ── 상태 → 표시값 매핑 ─────────────────────────────────────────

# 상태별 라벨 (HUD/배너에 그대로 노출)
STATE_TEXT: dict[AccStatus, str] = {
    AccStatus.OFF:       "OFF",
    AccStatus.STANDBY:   "STANDBY",
    AccStatus.CRUISING:  "CRUISING",
    AccStatus.FOLLOWING: "FOLLOWING",
    AccStatus.FAULT:     "FAULT",
}

# 패널/버튼용 색 (차분한 톤)
CLR: dict[AccStatus, str] = {
    AccStatus.OFF:       "#5F5E5A",
    AccStatus.STANDBY:   "#BA7517",
    AccStatus.CRUISING:  "#1D9E75",
    AccStatus.FOLLOWING: "#378ADD",
    AccStatus.FAULT:     "#E24B4A",
}

# HUD 용 색 (밝은 톤)
CLR_HUD: dict[AccStatus, str] = {
    AccStatus.OFF:       "#9E9D99",
    AccStatus.STANDBY:   "#F0A030",
    AccStatus.CRUISING:  "#3FFFB0",
    AccStatus.FOLLOWING: "#60B8FF",
    AccStatus.FAULT:     "#FF6B6B",
}

# 거리 단계별 색 (1=가까움/빨강, 3=멀리/녹색)
DIST_CLR: dict[int, str] = {1: "#E24B4A", 2: "#EF9F27", 3: "#1D9E75"}


# ── 상태 변환/검사 ──────────────────────────────────────────────

def status_from_int(raw: int) -> AccStatus:
    """`AccInfo.status` (int) → `AccStatus` 안전 변환. 알 수 없으면 FAULT."""
    try:
        return AccStatus(raw)
    except ValueError:
        return AccStatus.FAULT


def is_active(status: AccStatus) -> bool:
    """ACC 가 throttle 제어 중인지 (CRUISING/FOLLOWING)."""
    return status in ACTIVE_STATUSES


# ── _refresh 에서 추출한 상태 → 화면값 헬퍼 ────────────────────

def is_override_visible(status: AccStatus, brake_pressed: bool, accel_pwm: int) -> bool:
    """OVERRIDE 표시 여부. STANDBY/CRUISING/FOLLOWING 에서 페달 인가 시 True."""
    override = bool(brake_pressed) or int(accel_pwm) > 5
    return override and status in _OVERRIDE_VISIBLE_STATUSES


def pwm_display_pct(
    status: AccStatus,
    current_speed: float,
    set_speed: int,
    user_accel_pwm: int,
) -> float:
    """PWM 게이지에 표시할 % 값.
    ACC 액티브 + 목표 속도 양수일 때는 `current/set * 50` 근사치, 그 외에는 운전자 PWM."""
    if is_active(status) and set_speed > 0:
        return min(100.0, max(0.0, float(current_speed) / float(set_speed) * 50.0))
    return float(user_accel_pwm)


def fault_banner_text(status: AccStatus, ecu, can_connected: bool) -> str:
    """FAULT 배너에 표시할 문구. 없으면 빈 문자열 (배너 hide).

    SWR019: FAULT 진입 또는 실제 CAN 연결 중 HB lost / 에러코드 수신 시 표시.
    시뮬 모드 (`is_connected() == False`) 에서는 HB 체크를 건너뛴다.
    """
    if status == AccStatus.FAULT:
        if ecu.error_code:
            return f"FAULT  |  ECU error 0x{ecu.error_code:02X}  |  press ACC to clear"
        return "FAULT  |  press ACC to clear"
    if not can_connected:
        return ""
    if not ecu.heartbeat_ok:
        return "ECU HEARTBEAT LOST  |  checking CAN link…"
    if ecu.error_code:
        return f"ECU error 0x{ecu.error_code:02X}"
    return ""


def link_indicator(can_connected: bool, heartbeat_ok: bool) -> tuple[str, str]:
    """HUD CAN 링크 라벨의 (text, color)."""
    if not can_connected:
        return ("○ CAN sim", "#8A95A3")
    if heartbeat_ok:
        return ("● CAN", "#1D9E75")
    return ("● CAN · no HB", "#EF9F27")


def gap_color(status: AccStatus, level: int) -> str | None:
    """GAP 아이콘 색. OFF 면 None (회색). 그 외엔 거리 단계별 색."""
    if status == AccStatus.OFF:
        return None
    return DIST_CLR.get(level, "#EF9F27")


def set_speed_label(status: AccStatus, set_speed: int, active: bool) -> tuple[str, str]:
    """HUD TARGET 영역의 (text, color). STANDBY/액티브 → 'SET N cm/s', 그 외 → 'SET —'."""
    if active or status == AccStatus.STANDBY:
        color = CLR[status] if active else "#8A95A3"
        return (f"SET {int(set_speed)} cm/s", color)
    return ("SET —", "#8A95A3")
