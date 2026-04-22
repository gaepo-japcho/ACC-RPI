"""
HMI 렌더링용 상수 및 버튼 가용성 테이블.

상태·속도·거리단계는 `acc_can` 모듈(CanInterface)이 소유한다. HMI 는 50ms 주기로
`get_acc_info() / get_vehicle_info() / get_ecu_status()` 를 폴링해 렌더링만 수행한다.
로컬 상태머신은 두지 않는다.

Ref: STK020, STK021, SYS019, SYS029, SYS032, SYS034, SWR018, SWR028, SWR031
"""
from interfaces.acc_status import AccStatus


# 활성 상태 집합
ACTIVE_STATUSES = frozenset({AccStatus.CRUISING, AccStatus.FOLLOWING})


# 버튼 가용성 (SYS029 / STK021)
# key 는 AccStatus, value 는 버튼 식별자 → 활성 여부
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


# 1/10 스케일카 속도 파라미터 (단위: cm/s, SYS032 / STK024 / SWR031)
MIN_SET_SPEED_CMS = 5         # 최소 설정 속도
SPEED_INCREMENT_CMS = 1       # SPD+/- 클릭당 증감량
SPEED_GAUGE_MAX_CMS = 200     # 속도 게이지 최대 눈금
DEFAULT_DISTANCE_LEVEL = 3    # 기본 거리 단계 (SYS034)


def status_from_int(raw: int) -> AccStatus:
    """`AccInfo.status` (int) → `AccStatus` 안전 변환. 알 수 없으면 FAULT."""
    try:
        return AccStatus(raw)
    except ValueError:
        return AccStatus.FAULT
