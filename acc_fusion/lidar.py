import time
import numpy as np
from dataclasses import dataclass, field
from rplidar import RPLidar as _RPLidar


@dataclass
class LidarScan:
    points: np.ndarray  # shape: (N, 2) — columns: [angle_deg, distance_mm]
    timestamp: float = field(default_factory=time.time)
    scan_id: int = 0


class LidarReader:
    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200):
        """
        Args:
            port: RPLidar 시리얼 포트
            baudrate: 통신 속도 (A1/A2: 115200, S1: 256000)
        """
        self.port = port
        self.baudrate = baudrate

        self._lidar: _RPLidar | None = None
        self._scan_id = 0

    def open(self):
        self._lidar = _RPLidar(self.port, baudrate=self.baudrate)
        self._lidar.connect()

    def read(self) -> LidarScan | None:
        if self._lidar is None:
            return None

        for scan in self._lidar.iter_scans():
            # scan: list of (quality, angle, distance)
            arr = np.array([[angle, distance] for _, angle, distance in scan], dtype=np.float32)
            self._scan_id += 1
            return LidarScan(points=arr, timestamp=time.time(), scan_id=self._scan_id)

        return None

    def close(self):
        if self._lidar is not None:
            self._lidar.stop()
            self._lidar.disconnect()
            self._lidar = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()

    @property
    def is_open(self) -> bool:
        return self._lidar is not None
