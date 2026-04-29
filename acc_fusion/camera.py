import time
import numpy as np
from dataclasses import dataclass, field
from common.singleton import Singleton
from common import get_logger

log = get_logger(__name__)


@dataclass
class CameraFrame:
    image: np.ndarray
    timestamp: float = field(default_factory=time.time)
    frame_id: int = 0


class CameraReader(metaclass=Singleton):
    def __init__(
        self,
        source: int | str = 0,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
    ):
        """
        Args:
            source: 카메라 인덱스(int) 또는 영상 파일 경로(str) — Picamera2 사용 시 무시됨
            width: 캡처 해상도 너비
            height: 캡처 해상도 높이
            fps: 목표 FPS
        """
        self.source = source
        self.width = width
        self.height = height
        self.fps = fps

        self._cam = None
        self._frame_id = 0

    def open(self) -> bool:
        try:
            from picamera2 import Picamera2
            self._cam = Picamera2()
            # FrameDurationLimits 가 fps 의 진실의 원천 (us 단위, min/max 동일값으로 고정).
            # config.toml 의 fps 인자는 여기로 전달 — sensor 가 cap 을 무시하면 detect/yolo
            # 로그가 cap 보다 높게 찍히므로 적용 후 확인 필요.
            frame_duration_us = int(1_000_000 / self.fps)
            config = self._cam.create_preview_configuration(
                main={"size": (self.width, self.height), "format": "RGB888"},
                buffer_count=2,
                controls={"FrameDurationLimits": (frame_duration_us, frame_duration_us)},
            )
            self._cam.configure(config)
            self._cam.start()
            log.info(f"카메라 열기 성공 (Picamera2): {self.width}x{self.height} @{self.fps}fps")
            return True
        except Exception as e:
            log.error(f"카메라 열기 실패: {e}")
            return False

    def read(self) -> CameraFrame | None:
        if self._cam is None:
            return None
        try:
            frame = self._cam.capture_array()
            self._frame_id += 1
            return CameraFrame(image=frame, timestamp=time.time(), frame_id=self._frame_id)
        except Exception as e:
            log.warning(f"카메라 프레임 읽기 실패: {e}")
            return None

    def close(self):
        if self._cam is not None:
            self._cam.stop()
            self._cam = None
            log.info("카메라 닫힘")

    def __enter__(self):
        if not self.open():
            raise RuntimeError(f"카메라를 열 수 없습니다: {self.source}")
        return self

    def __exit__(self, *_):
        self.close()

    @classmethod
    def from_config(cls) -> "CameraReader":
        from common import config

        c = config["camera"]
        return cls(
            source=c["source"],
            width=c["width"],
            height=c["height"],
            fps=c["fps"],
        )

    @property
    def is_open(self) -> bool:
        return self._cam is not None
