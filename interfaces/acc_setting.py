from dataclasses import dataclass


@dataclass
class AccSetting:
    set_speed: int  # cm/s (-500~500 — DBC SET_ACC_SPD int16)
    distance_level: int  # 1/2/3 (1=CLOSE, 2=MEDIUM, 3=FAR)
