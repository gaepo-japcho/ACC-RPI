from dataclasses import dataclass


@dataclass
class PedalInput:
    brake: bool  # 브레이크 눌림 여부 (DBC BTN_BRAKE, level-triggered)
    accel_pwm: int  # PWM (-128~127 — DBC SET_ACCEL_PWM int8, 음수=reverse)
