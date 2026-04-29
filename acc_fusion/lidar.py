import time
import numpy as np
from dataclasses import dataclass, field
from rplidar import RPLidar as _RPLidar
from common.singleton import Singleton
from common import get_logger

log = get_logger(__name__)


@dataclass
class LidarScan:
    points: np.ndarray  # shape: (N, 2) — columns: [angle_deg, distance_mm]
    timestamp: float = field(default_factory=time.time)
    scan_id: int = 0


class LidarReader(metaclass=Singleton):
    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        angle_offset: float = 0.0,
        reverse: bool = False,
    ):
        """
        Args:
            port: RPLidar 시리얼 포트
            baudrate: 통신 속도 (A1/A2: 115200, S1: 256000)
            angle_offset: 설치 방향 보정값 (도). 반시계 방향 기준으로 더해짐
            reverse: True면 회전 방향 반전 (라이다를 뒤집어 설치한 경우)
        """
        self.port = port
        self.baudrate = baudrate
        self.angle_offset = angle_offset
        self.reverse = reverse

        self._lidar: _RPLidar | None = None
        self._iterator = None
        self._scan_id = 0

    def open(self) -> None:
        self._lidar = _RPLidar(self.port, self.baudrate)
        self._iterator = self._lidar.iter_scans()
        log.info(f"LiDAR 연결: port={self.port} baudrate={self.baudrate}")

    def _to_array(self, scan) -> np.ndarray:
        # rplidar iter_scans 는 (quality, angle, distance) 튜플 리스트 반환.
        # Python list comprehension 대신 np.asarray 한방으로 C loop 활용.
        if not scan:
            return np.empty((0, 2), dtype=np.float32)
        arr = np.asarray(scan, dtype=np.float32)  # (N, 3)
        return arr[:, 1:3]  # angle, distance 컬럼만

    def _apply_transform(self, arr: np.ndarray) -> np.ndarray:
        # config 가 identity (offset=0, reverse=false) 면 copy/연산 전부 스킵.
        if not self.reverse and self.angle_offset == 0.0:
            return arr
        angles = arr[:, 0]
        if self.reverse:
            angles = (360.0 - angles) % 360.0
        if self.angle_offset != 0.0:
            angles = (angles + self.angle_offset) % 360.0
        result = arr.copy()
        result[:, 0] = angles
        return result

    def raw_read(self) -> LidarScan | None:
        """가공 없이 센서 원본값 그대로 반환"""
        if self._iterator is None:
            return None

        scan = next(self._iterator, None)
        if scan is None:
            return None

        arr = self._to_array(scan)
        self._scan_id += 1
        return LidarScan(points=arr, timestamp=time.time(), scan_id=self._scan_id)

    def read(self) -> LidarScan | None:
        """angle_offset, reverse 적용 후 반환"""
        scan = self.raw_read()
        if scan is None:
            return None
        return LidarScan(
            points=self._apply_transform(scan.points),
            timestamp=scan.timestamp,
            scan_id=scan.scan_id,
        )

    def close(self):
        if self._lidar is not None:
            self._iterator = None
            self._lidar.stop()
            self._lidar.disconnect()
            self._lidar = None
            log.info(f"LiDAR 연결 해제: port={self.port}")

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()

    @classmethod
    def from_config(cls) -> "LidarReader":
        from common import config

        c = config["lidar"]
        return cls(
            port=c["port"],
            baudrate=c["baudrate"],
            angle_offset=c["angle_offset"],
            reverse=c["reverse"],
        )

    @property
    def is_open(self) -> bool:
        return self._lidar is not None
