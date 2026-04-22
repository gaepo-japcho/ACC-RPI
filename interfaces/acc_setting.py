from dataclasses import dataclass


@dataclass
class AccSetting:
    set_speed: int  # cm/s 절대값
    distance_level: int  # 1/2/3
