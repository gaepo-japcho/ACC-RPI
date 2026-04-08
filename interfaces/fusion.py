from dataclasses import dataclass

@dataclass
class FusionData:
    detected: bool  # 전방 차량 감지 여부
    distance: int   # 전방 차량 거리 (cm)
