__version__ = "0.0.1"
__author__ = "parkjbdev"

import threading
import numpy as np

from common.singleton import Singleton
from interfaces.fusion import FusionData

__all__ = ["FusionData", "Fusion"]

_VEHICLE_CLASSES = {"car", "truck", "bus", "motorcycle"}
_FRONT_ANGLE_DEG = 30  # 전방으로 인정할 좌우 범위 (±30°)



class Fusion(metaclass=Singleton):
    def __init__(self):
        from .camera import CameraReader
        from .lidar import LidarReader, LidarScan
        from .yolo import YOLODetector

        self._cam = CameraReader.from_config()
        self._lidar = LidarReader.from_config()
        self._detector = YOLODetector.from_config()

        self._latest_vehicles: list[dict] = []
        self._latest_scan: LidarScan | None = None
        self._vehicles_lock = threading.Lock()
        self._scan_lock = threading.Lock()

        self._stop_event = threading.Event()
        self._camera_thread = threading.Thread(
            target=self._camera_worker, daemon=True, name="fusion-camera"
        )
        self._lidar_thread = threading.Thread(
            target=self._lidar_worker, daemon=True, name="fusion-lidar"
        )

    def __enter__(self) -> "Fusion":
        self._cam.open()
        self._lidar.open()
        self._camera_thread.start()
        self._lidar_thread.start()
        return self

    def __exit__(self, *_) -> None:
        self._stop_event.set()
        self._camera_thread.join(timeout=2.0)
        self._lidar_thread.join(timeout=2.0)
        self._cam.close()
        self._lidar.close()

    def _camera_worker(self) -> None:
        while not self._stop_event.is_set():
            frame = self._cam.read()
            if frame is None:
                continue
            detections = self._detector.detect(frame.image)
            vehicles = [d for d in detections if d["class_name"] in _VEHICLE_CLASSES]
            with self._vehicles_lock:
                self._latest_vehicles = vehicles

    def _lidar_worker(self) -> None:
        while not self._stop_event.is_set():
            scan = self._lidar.read()
            if scan is None:
                continue
            with self._scan_lock:
                self._latest_scan = scan

    def update(self) -> FusionData:
        """
        Fusion 모듈이 산출한 전방 차량 데이터를 반환합니다.

        호출 시점: Fusion 모듈의 센서 처리 완료 시 (비동기)
        호출 주체: Fusion 모듈

        return:
            FusionData:
                detected (bool) — 전방 차량 감지 여부
                distance (int)  — 전방 차량 거리 (cm)

        동작:
            - CAN 모듈은 내부 버퍼에 최신값 유지
            - 다음 20ms TX 주기에 Raspi→ECU 메시지의 VEH_DET, VEH_DIST에 포함
            - 일정 시간(예: 100ms) 미갱신 시 ERR_SENSOR 플래그 설정
        """
        with self._vehicles_lock:
            vehicles = self._latest_vehicles.copy()

        if not vehicles:
            return FusionData(detected=False, distance=0)

        with self._scan_lock:
            scan = self._latest_scan

        if scan is None or len(scan.points) == 0:
            return FusionData(detected=True, distance=0)

        angles = scan.points[:, 0]
        front_mask = (angles <= _FRONT_ANGLE_DEG) | (angles >= 360 - _FRONT_ANGLE_DEG)
        front_points = scan.points[front_mask]

        if len(front_points) == 0:
            return FusionData(detected=True, distance=0)

        distance_cm = int(np.min(front_points[:, 1]) / 10)
        return FusionData(detected=True, distance=distance_cm)

    def update_fusion_data(self) -> FusionData:
        """update()와 동일, 이름만 다름"""
        return self.update()
