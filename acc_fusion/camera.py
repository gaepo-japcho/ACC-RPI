import cv2
import time
import numpy as np
from dataclasses import dataclass, field
from common.singleton import Singleton


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
            source: 카메라 인덱스(int) 또는 영상 파일 경로(str)
            width: 캡처 해상도 너비
            height: 캡처 해상도 높이
            fps: 목표 FPS
        """
        self.source = source
        self.width = width
        self.height = height
        self.fps = fps

        self._cap: cv2.VideoCapture | None = None
        self._frame_id = 0

    def open(self) -> bool:
        self._cap = cv2.VideoCapture(self.source)
        if not self._cap.isOpened():
            return False

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS, self.fps)
        return True

    def read(self) -> CameraFrame | None:
        if self._cap is None or not self._cap.isOpened():
            return None

        ret, frame = self._cap.read()
        if not ret:
            return None

        self._frame_id += 1
        return CameraFrame(image=frame, timestamp=time.time(), frame_id=self._frame_id)

    def close(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def __enter__(self):
        if not self.open():
            raise RuntimeError(f"카메라를 열 수 없습니다: {self.source}")
        return self

    def __exit__(self, *_):
        self.close()

    @classmethod
    def from_config(cls) -> "CameraReader":
        from .config import config

        c = config["camera"]
        return cls(
            source=c["source"],
            width=c["width"],
            height=c["height"],
            fps=c["fps"],
        )

    @property
    def is_open(self) -> bool:
        return self._cap is not None and self._cap.isOpened()
