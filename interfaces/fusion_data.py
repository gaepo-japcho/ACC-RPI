from dataclasses import dataclass


@dataclass
class FusionData:
    detected: bool  # 전방 차량 감지 여부
    distance: int  # 전방 차량 거리 (mm, 0~12000 — DBC VEH_DIST uint16, RPLiDAR 12m 범위)
