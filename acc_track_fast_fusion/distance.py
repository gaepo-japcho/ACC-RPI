"""
LiDAR 매핑 거리의 시간적 smoothing.

ACC 시나리오 가정: 전방 차량이 한 update 사이클 안에 텔레포트하지 않음.
LiDAR raw glitch / 부분 가림 / bbox 살짝 흔들림으로 인한 거리 jitter 를 두 layer 로 잡음:

  1. Slew rate limit — 한 step 변화량을 max_step_mm 으로 cap.
     push 50Hz 기준 300mm/step = 15m/s = 54km/h 의 상대속도 변화율 마진.
     실제 차량 상대속도가 그보다 빠르게 바뀌는 일은 거의 없으니 명백한 glitch 만 거름.
  2. EMA blend — 작은 jitter 를 가중평균으로 부드럽게.

트랙 ID 가 바뀌면 reset → 새 트랙의 첫 raw 값으로 출발 (smoothing 없이).
"""


class DistanceSmoother:
    def __init__(self, alpha: float = 0.4, max_step_mm: int = 300):
        """
        alpha: EMA 가중치 (raw 가 차지하는 비율). 클수록 raw 추종, 작을수록 부드러움.
               0.4 = 새 값 40%, 이전 60%. ACC 응답성과 안정성 trade-off.
        max_step_mm: 한 step 변화량 상한. 이 값 넘는 입력은 cap 처리 (slew rate limit).
        """
        self._alpha = alpha
        self._max_step = max_step_mm
        self._last_track_id: int | None = None
        self._smoothed: float | None = None

    def step(self, track_id: int, raw_mm: int) -> int:
        """
        새 raw 측정값을 받아 smoothed 거리 반환.
        track_id 가 바뀌면 (또는 첫 호출이면) raw 그대로 시작.
        """
        if track_id != self._last_track_id or self._smoothed is None:
            self._last_track_id = track_id
            self._smoothed = float(raw_mm)
            return int(self._smoothed)

        # 1. Slew rate limit: 변화량 cap
        delta = raw_mm - self._smoothed
        if delta > self._max_step:
            delta = self._max_step
        elif delta < -self._max_step:
            delta = -self._max_step
        capped = self._smoothed + delta

        # 2. EMA blend
        self._smoothed = self._alpha * capped + (1 - self._alpha) * self._smoothed
        return int(self._smoothed)

    def reset(self) -> None:
        """트랙이 사라졌을 때 호출. 다음 트랙의 첫 raw 부터 새로 시작."""
        self._last_track_id = None
        self._smoothed = None
