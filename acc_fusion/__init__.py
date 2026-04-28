__version__ = "0.0.1"
__author__ = "parkjbdev"

import threading
import numpy as np

from common.singleton import Singleton
from common import get_logger, config
from interfaces.fusion_data import FusionData
from acc_can import CanInterface
from acc_can._dbc import msg

log = get_logger(__name__)

__all__ = ["FusionData", "Fusion"]

# SENSOR_FUSION TX 주기에 맞춰 push (DBC GenMsgCycleTime).
_PUSH_PERIOD_SEC = msg("SENSOR_FUSION").cycle_time / 1000

# LiDAR 노이즈/outlier 필터
_LIDAR_MIN_VALID_MM = 100      # 이 이하 거리는 센서 본체/케이블 반사로 간주, 제거
_LIDAR_MIN_VALID_POINTS = 3    # 이 미만이면 신뢰 불가 — 거리 0 으로 보고
_LIDAR_DIST_PERCENTILE = 5     # outlier 회피용 하위 분위수 (np.min 대신)


class Fusion(metaclass=Singleton):
    def __init__(self, can: CanInterface):
        from .camera import CameraReader
        from .lidar import LidarReader, LidarScan
        from .yolo import YOLODetector

        self._can = can
        self._cam = CameraReader.from_config()
        self._lidar = LidarReader.from_config()
        self._detector = YOLODetector.from_config()
        self._cam_hfov = float(config["camera"]["hfov"])

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
            vehicles = self._detector.detect(frame.image)
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

    def _pixel_to_angle(self, x: float) -> float:
        """카메라 픽셀 x → 각도 (도, 화면 중심=0°, 우측=+).

        HFOV 안에서 선형 매핑 — 광각 왜곡 보정은 안 함 (FOV 75°까지는 충분히 선형).
        """
        return (x - self._cam.width / 2.0) / self._cam.width * self._cam_hfov

    def update(self) -> FusionData:
        """
        최신 카메라/LiDAR 상태로부터 전방 차량 FusionData 산출.

        알고리즘:
            1. YOLO bbox 의 좌우 픽셀 → 카메라 HFOV 기반 각도 범위
            2. 그 각도 범위에 들어오는 LiDAR 점만 거리 후보
            3. 차량 여러 대면 모든 bbox 마스크의 union → 그 중 최솟값

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

        # bbox 마다 각도 범위 (signed) 계산 → LiDAR 좌표 (0~360) 로 변환 → mask union
        angles = scan.points[:, 0]  # 0~360, lidar.py 의 angle_offset 적용된 값
        mask = np.zeros(len(angles), dtype=bool)
        for v in vehicles:
            x1, _, x2, _ = v["bbox"]
            a_left = self._pixel_to_angle(x1) % 360.0
            a_right = self._pixel_to_angle(x2) % 360.0
            # bbox 가 정면(0°) 을 포함하면 a_left > a_right (예: 350° ~ 10°)
            if a_left <= a_right:
                mask |= (angles >= a_left) & (angles <= a_right)
            else:
                mask |= (angles >= a_left) | (angles <= a_right)

        bbox_points = scan.points[mask]
        # 센서 노이즈 (본체/케이블 반사 등) 제거
        valid = bbox_points[bbox_points[:, 1] >= _LIDAR_MIN_VALID_MM]
        if len(valid) < _LIDAR_MIN_VALID_POINTS:
            log.debug(
                f"전방 차량 감지, bbox 유효 점 부족: "
                f"{len(valid)}/{_LIDAR_MIN_VALID_POINTS}"
            )
            return FusionData(detected=True, distance=0)

        # lidar.py 가 [angle_deg, distance_mm] 컬럼으로 반환 (RPLiDAR raw=mm).
        # np.min 은 노이즈 한 점에 거리 결정됨 → percentile 로 outlier 강건성 확보.
        distance_mm = int(np.percentile(valid[:, 1], _LIDAR_DIST_PERCENTILE))
        log.debug(
            f"전방 차량 감지: vehicles={len(vehicles)} "
            f"bbox점={len(bbox_points)} 유효={len(valid)} distance={distance_mm}mm"
        )
        return FusionData(detected=True, distance=distance_mm)
