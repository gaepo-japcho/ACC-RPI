"""
ACC Project — CAN 메시지 encode/decode (pure functions)
=========================================================
Python 값 ↔ CAN 페이로드 변환. 비트/바이트 레이아웃은 모두
`_dbc.msg(...).encode/decode` (cantools) 에 위임하며, 여기선
dataclass ↔ signals dict 마샬링만 수행한다.

Pure function (스레드/락/버스 의존 없음) → 단위 테스트가 자유롭다.

TX encoders (Python obj → bytes):
  encode_veh_ctrl        0x120 VEH_CTRL        brake + accel_pwm
  encode_acc_ctrl        0x510 ACC_CTRL        ACC 버튼 + 목표 속도/레벨
  encode_sensor_fusion   0x110 SENSOR_FUSION   전방 차량 감지/거리
  encode_sensor_heartbeat 0x111 SENSOR_HEARTBEAT  HB + ERR

RX decoders (bytes → Python obj):
  decode_acc_status      0x520 ACC_STATUS     → AccInfo
  decode_ecu_heartbeat   0x410 ECU_HEARTBEAT  → err_ecu (int)
  decode_mtr_spd_fb      0x300 MTR_SPD_FB     → 자차 평균속도 (float cm/s)
"""
from interfaces.acc_info import AccInfo
from interfaces.acc_setting import AccSetting
from interfaces.button_input import ButtonInput
from interfaces.fusion_data import FusionData
from interfaces.pedal_input import PedalInput

from . import _dbc


# ─── TX encoders (Python obj → bytes) ───────────────────────────────────────

def encode_veh_ctrl(pedal: PedalInput) -> bytes:
    """0x120 VEH_CTRL (20ms) — brake + accel PWM."""
    return _dbc.msg("VEH_CTRL").encode({
        "SET_ACCEL_PWM": int(pedal.accel_pwm),
        "BTN_BRAKE":     int(pedal.brake),
    })


def encode_acc_ctrl(buttons: ButtonInput, setting: AccSetting) -> bytes:
    """0x510 ACC_CTRL (50ms) — ACC 버튼 (edge trigger) + 목표 속도/레벨."""
    return _dbc.msg("ACC_CTRL").encode({
        "BTN_ACC_OFF":    int(buttons.btn_acc_off),
        "BTN_ACC_SET":    int(buttons.btn_acc_set),
        "BTN_ACC_RES":    int(buttons.btn_acc_res),
        "BTN_ACC_CANCEL": int(buttons.btn_acc_cancel),
        "SET_ACC_SPD":    int(setting.set_speed),
        "SET_ACC_LVL":    int(setting.distance_level),
    })


def encode_sensor_fusion(fusion: FusionData) -> bytes:
    """0x110 SENSOR_FUSION (20ms) — 전방 차량 감지 + 거리(mm)."""
    return _dbc.msg("SENSOR_FUSION").encode({
        "VEH_DET":  int(fusion.detected),
        "VEH_DIST": int(fusion.distance),
    })


def encode_sensor_heartbeat(hb_counter: int, err_sensor: int) -> bytes:
    """0x111 SENSOR_HEARTBEAT (20ms) — HB 카운터 + ERR 플래그."""
    return _dbc.msg("SENSOR_HEARTBEAT").encode({
        "HB_SENSOR":  int(hb_counter),
        "ERR_SENSOR": int(err_sensor),
    })


# ─── RX decoders (bytes → Python obj) ───────────────────────────────────────

def decode_acc_status(data: bytes) -> AccInfo:
    """
    0x520 ACC_STATUS → AccInfo.

    decode_choices=False: DBC VAL_ 정의가 있는 GET_ACC_STATE 를 NamedSignalValue
    로 변환하지 않고 raw int 그대로 반환. AccInfo.status 가 int 라 필요.
    """
    signals = _dbc.msg("ACC_STATUS").decode(data, decode_choices=False)
    return AccInfo(
        status=int(signals["GET_ACC_STATE"]),
        set_speed=int(signals["GET_ACC_SPD"]),
        distance_level=int(signals["GET_ACC_LVL"]),
    )


def decode_ecu_heartbeat(data: bytes) -> int:
    """0x410 ECU_HEARTBEAT → err_ecu (0=정상, 그 외=에러)."""
    signals = _dbc.msg("ECU_HEARTBEAT").decode(data, decode_choices=False)
    return int(signals["ERR_ECU"])


def decode_mtr_spd_fb(data: bytes) -> float:
    """
    0x300 MTR_SPD_FB → 자차 평균속도 (cm/s).

    cantools 가 DBC factor 0.02 를 자동 적용해 scaled float 반환.
    GET_SPD_LF/RF/LR/RR (개별 휠 속도) 는 ECU-only 수신자라 SENSOR 는 무시.
    """
    signals = _dbc.msg("MTR_SPD_FB").decode(data, decode_choices=False)
    return float(signals["GET_SPD_AVG"])
