__version__ = "0.0.1"
__author__ = "parkjbdev"

import threading
import time
import numpy as np

from common.singleton import Singleton
from common import get_logger
from interfaces.fusion_data import FusionData
from acc_can import CanInterface
from acc_can._dbc import msg

log = get_logger(__name__)

__all__ = ["FusionData", "Fusion"]

_FRONT_ANGLE_DEG = 30  # 전방으로 인정할 좌우 범위 (±30°)
# SENSOR_FUSION TX 주기에 맞춰 push (DBC GenMsgCycleTime).
_PUSH_PERIOD_SEC = msg("SENSOR_FUSION").cycle_time / 1000


class Fusion(metaclass=Singleton):
    def __init__(self, can: CanInterface):
        from .camera import CameraFrame, CameraReader
        from .lidar import LidarReader, LidarScan
        from .yolo import YOLODetector

        self._can = can
        self._cam = CameraReader.from_config()
        self._lidar = LidarReader.from_config()
        self._detector = YOLODetector.from_config()

        # Capture 워커가 슬롯에 최신 프레임만 덮어쓰고, Detect 워커가 Event 기다렸다가 집어감.
        # 옛 프레임은 자동 폐기 — 검출이 capture 보다 느려도 누적 안 됨.
        self._latest_frame: CameraFrame | None = None
        self._latest_vehicles: list[dict] = []
        self._latest_scan: LidarScan | None = None
        self._latest_annotated: np.ndarray | None = None  # show_window=True 일 때만 채움
        self._frame_lock = threading.Lock()
        self._vehicles_lock = threading.Lock()
        self._scan_lock = threading.Lock()
        self._annotated_lock = threading.Lock()
        self._frame_ready = threading.Event()

        self._stop_event = threading.Event()
        self._capture_thread: threading.Thread | None = None
        self._detect_thread: threading.Thread | None = None
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
        self._frame_ready.clear()
        self._capture_thread = threading.Thread(
            target=self._capture_worker, daemon=True, name="fusion-capture"
        )
        self._detect_thread = threading.Thread(
            target=self._detect_worker, daemon=True, name="fusion-detect"
        )
        self._lidar_thread = threading.Thread(
            target=self._lidar_worker, daemon=True, name="fusion-lidar"
        )
        self._push_thread = threading.Thread(
            target=self._push_worker, daemon=True, name="fusion-push"
        )
        self._capture_thread.start()
        self._detect_thread.start()
        self._lidar_thread.start()
        self._push_thread.start()
        self._running = True
        log.info("Capture/Detect/LiDAR/Push 워커 스레드 시작")
        return self

    def __exit__(self, *_) -> None:
        if not self._running:
            return
        log.info("Fusion 종료 중...")
        self._stop_event.set()
        self._frame_ready.set()  # detect 워커가 wait() 에 묶여 있으면 깨움
        if self._capture_thread:
            self._capture_thread.join(timeout=2.0)
        if self._detect_thread:
            self._detect_thread.join(timeout=2.0)
        if self._lidar_thread:
            self._lidar_thread.join(timeout=2.0)
        if self._push_thread:
            self._push_thread.join(timeout=2.0)
        self._cam.close()
        self._lidar.close()
        self._running = False
        log.info("Fusion 종료 완료")

    def _capture_worker(self) -> None:
        """카메라 → 슬롯 (1 칸). 항상 최신 프레임만 보관, ISP 속도(~30Hz)로 자연 제한."""
        while not self._stop_event.is_set():
            frame = self._cam.read()
            if frame is None:
                log.warning("Capture 워커: 프레임 읽기 실패")
                continue
            with self._frame_lock:
                self._latest_frame = frame
            self._frame_ready.set()

    def _detect_worker(self) -> None:
        """슬롯에 새 프레임이 도착하면 YOLO 추론. Event 기반 — sleep 불필요."""
        last_log = time.monotonic()
        frames = 0
        det_total = 0
        last_seen_id = -1
        while not self._stop_event.is_set():
            if not self._frame_ready.wait(timeout=0.5):
                continue
            self._frame_ready.clear()
            with self._frame_lock:
                frame = self._latest_frame
            if frame is None or frame.frame_id == last_seen_id:
                continue
            last_seen_id = frame.frame_id
            vehicles = self._detector.detect(frame.image)
            with self._vehicles_lock:
                self._latest_vehicles = vehicles
            # show_window=True 면 HMI 가 가져갈 수 있도록 bbox 그려서 버퍼에 저장.
            # cv2.imshow 는 PyQt5 와 동일 프로세스에서 충돌하므로 사용하지 않는다.
            if self._detector.show_window:
                annotated = frame.image.copy()
                self._detector.draw(annotated, vehicles)
                with self._annotated_lock:
                    self._latest_annotated = annotated
            frames += 1
            det_total += len(vehicles)
            now = time.monotonic()
            if now - last_log >= 1.0:
                log.info(
                    f"detect/yolo: {frames} fps, vehicles avg={det_total/frames:.2f}"
                )
                last_log = now
                frames = 0
                det_total = 0

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

    def get_annotated_frame(self) -> np.ndarray | None:
        """show_window=True 일 때 _camera_worker 가 채워둔 bbox 그려진 RGB 프레임 사본."""
        with self._annotated_lock:
            return None if self._latest_annotated is None else self._latest_annotated.copy()

    @property
    def show_window(self) -> bool:
        return self._detector.show_window

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
