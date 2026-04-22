from dataclasses import dataclass


@dataclass
class AccInfo:
    status: int  # 0=OFF,1=STANDBY,2=CRUISING,3=FOLLOWING,4=FAULT
    set_speed: int  # cm/s
    distance_level: int  # 1/2/3
