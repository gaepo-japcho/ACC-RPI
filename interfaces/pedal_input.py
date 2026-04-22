from dataclasses import dataclass


@dataclass
class PedalInput:
    brake: bool  # 0% 초과 시 True
    accel_pwm: int  # 0~100 (%)
