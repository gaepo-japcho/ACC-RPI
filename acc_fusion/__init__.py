__version__ = "0.0.1"
__author__ = "parkjbdev"

import threading
import numpy as np

from common.singleton import Singleton
from common import get_logger
from interfaces.fusion_data import FusionData
from acc_can import CanInterface
from acc_can._dbc import msg

log = get_logger(__name__)

__all__ = ["FusionData", "Fusion"]

_VEHICLE_CLASSES = {"car", "truck", "bus", "motorcycle"}
_FRONT_ANGLE_DEG = 30  # 전방으로 인정할 좌우 범위 (±30°)
# SENSOR_FUSION TX 주기에 맞춰 push (DBC GenMsgCycleTime).
_PUSH_PERIOD_SEC = msg("SENSOR_FUSION").cycle_time / 1000


class Fusion(metaclass=Singleton):
    def __init__(self, can: CanInterface):
        from .camera import CameraReader
        from .lidar import LidarReader, LidarScan
        from .yolo import YOLODetector

        self._can = can
        self._cam = CameraReader.from_config()
        self._lidar = LidarReader.from_config()
        self._detector = YOLODetector.from_config()

        self._latest_vehicles: list[dict] = []
        self._latest_scan: LidarScan | None = None
        self._vehicles_lock = threading.Lock()
        self._scan_lock = threading.Lock()

        self._stop_event = threading.Event()
        self._camera_thread: threading.Thread | None = None
        self._lidar_thread: threading.Thread | None = None
        self._push_thread: threading.Thread | None = None
        self._running = False

    def __enter__(self) -> "Fusion":
        if self._running:
            log.warning("Fusion already running.")
            return self
        log.info("Fusion 시작")
        self._cam.open()
        self._lidar.open()
        self._stop_event.clear()
        self._camera_thread = threading.Thread(
            target=self._camera_worker, daemon=True, name="fusion-camera"
        )
        self._lidar_thread = threading.Thread(
            target=self._lidar_worker, daemon=True, name="fusion-lidar"
        )
        self._push_thread = threading.Thread(
            target=self._push_worker, daemon=True, name="fusion-push"
        )
        self._camera_thread.start()
        self._lidar_thread.start()
        self._push_thread.start()
        self._running = True
        log.info("카메라/LiDAR/Push 워커 스레드 시작")
        return self

    def __exit__(self, *_) -> None:
        if not self._running:
            return
        log.info("Fusion 종료 중...")
        self._stop_event.set()
        if self._camera_thread:
            self._camera_thread.join(timeout=2.0)
        if self._lidar_thread:
            self._lidar_thread.join(timeout=2.0)
        if self._push_thread:
            self._push_thread.join(timeout=2.0)
        self._cam.close()
        self._lidar.close()
        self._running = False
        log.info("Fusion 종료 완료")

    def _camera_worker(self) -> None:
        while not self._stop_event.is_set():
            frame = self._cam.read()
            if frame is None:
                log.warning("카메라 워커: 프레임 읽기 실패")
                continue
            detections = self._detector.detect(frame.image)
            vehicles = [d for d in detections if d["class_name"] in _VEHICLE_CLASSES]
            with self._vehicles_lock:
                self._latest_vehicles = vehicles

    def _lidar_worker(self) -> None:
        while not self._stop_event.is_set():
            scan = self._lidar.read()
            if scan is None:
                log.warning("LiDAR 워커: 스캔 읽기 실패")
                continue
            with self._scan_lock:
                self._latest_scan = scan

    def _push_worker(self) -> None:
        """SENSOR_FUSION 주기로 update() → CanInterface.update_fusion_data() push."""
        while not self._stop_event.wait(_PUSH_PERIOD_SEC):
            self._can.update_fusion_data(self.update())

    def update(self) -> FusionData:
        """
        최신 카메라/LiDAR 상태로부터 전방 차량 FusionData 산출.

        return:
            FusionData:
                detected (bool) — 전방 차량 감지 여부 (YOLO 분류 결과)
                distance (int)  — 전방 차량 최근접 거리 (mm, 0~12000 — DBC VEH_DIST)
        """
        with self._vehicles_lock:
            vehicles = self._latest_vehicles.copy()

        if not vehicles:
            return FusionData(detected=False, distance=0)

        with self._scan_lock:
            scan = self._latest_scan

        if scan is None or len(scan.points) == 0:
            log.debug("전방 차량 감지, LiDAR 데이터 없음")
            return FusionData(detected=True, distance=0)

        angles = scan.points[:, 0]
        front_mask = (angles <= _FRONT_ANGLE_DEG) | (angles >= 360 - _FRONT_ANGLE_DEG)
        front_points = scan.points[front_mask]

        if len(front_points) == 0:
            log.debug("전방 차량 감지, 전방 LiDAR 포인트 없음")
            return FusionData(detected=True, distance=0)

        # lidar.py 가 [angle_deg, distance_mm] 컬럼으로 반환 (RPLiDAR raw=mm).
        distance_mm = int(np.min(front_points[:, 1]))
        log.debug(f"전방 차량 감지: distance={distance_mm}mm")
        return FusionData(detected=True, distance=distance_mm)
