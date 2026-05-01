"""
acc_track_fast_fusion — acc_fast_fusion 위에 OpenCV 트래커 한 층 더.

차이점:
- acc_fast_fusion: YOLO 비동기, capture FPS 가 NPU 갱신률에 좌우됨
- acc_track_fast_fusion: 매 카메라 frame 마다 cv2 template tracker.update() 로 bbox 보간,
  YOLO 갱신 시 IoU 매칭으로 진실 보정. 차량 짧은 미인식 갭 메꿈.

정책:
- tracker.update() fail → 즉시 트랙 제거 (예측 grace 없음)
- YOLO 매칭 안 된 트랙 → 즉시 제거 (= 차량이 화면에서 사라지면 다음 YOLO 사이클 안에 bbox 사라짐)

LiDAR fusion:
- 각 트랙 bbox 의 x 범위를 카메라 hfov 로 각도 환산 → 해당 각도 범위의 LiDAR 점들
  중 최단 거리 채택. 여러 트랙 중 가장 가까운 차량 거리를 FusionData.distance 로 push.
"""
__version__ = "0.0.1"

import threading
import time

import numpy as np

from acc_can import CanInterface
from acc_can._dbc import msg
from common import config, get_logger
from common.singleton import Singleton
from interfaces.fusion_data import FusionData

log = get_logger(__name__)

__all__ = ["FusionData", "Fusion"]

_PUSH_PERIOD_SEC = msg("SENSOR_FUSION").cycle_time / 1000


def bbox_to_distance_mm(
    bbox: tuple, scan_points: np.ndarray, hfov_deg: float, img_w: int
) -> int | None:
    """
    bbox 의 x 범위를 카메라 hfov 로 각도 환산 → 해당 각도 영역의 LiDAR 점들 중 최단 거리.

    카메라 좌표: x=0 좌측, x=img_w 우측. 광축 = 이미지 중앙 = LiDAR 0°.
    LiDAR 각도 규약 (acc_fusion 과 동일): 0=정면, 시계 방향 0~360°. 좌측은 360°에 가까운 값.

    Args:
        bbox: (x1, y1, x2, y2) — 픽셀 좌표
        scan_points: shape (N, 2) — [angle_deg, distance_mm]
        hfov_deg: 카메라 수평 시야각 (config [camera] hfov)
        img_w: 카메라 가로 픽셀 수

    Returns:
        bbox 각도 범위 안에 LiDAR 점이 있으면 최단 distance_mm (int).
        없으면 None — fusion update() 에서 distance=0 처리.
    """
    if scan_points is None or len(scan_points) == 0:
        return None

    x1, _, x2, _ = bbox
    half_w = img_w / 2
    half_fov = hfov_deg / 2

    left_cam_deg = (x1 - half_w) / half_w * half_fov
    right_cam_deg = (x2 - half_w) / half_w * half_fov

    # 카메라 각도(±) → LiDAR 각도(0..360). 음수면 360 더해서 wrap.
    left_lidar = left_cam_deg if left_cam_deg >= 0 else 360.0 + left_cam_deg
    right_lidar = right_cam_deg if right_cam_deg >= 0 else 360.0 + right_cam_deg

    angles = scan_points[:, 0]
    if left_lidar <= right_lidar:
        mask = (angles >= left_lidar) & (angles <= right_lidar)
    else:
        # wrap-around: bbox 가 정면을 가로지르면 left=350, right=10 같은 형태
        mask = (angles >= left_lidar) | (angles <= right_lidar)

    in_box = scan_points[mask]
    if len(in_box) == 0:
        return None
    return int(np.min(in_box[:, 1]))


class Fusion(metaclass=Singleton):
    def __init__(self, can: CanInterface):
        from acc_fast_fusion.yolo import YOLODetectorAsync
        from acc_fusion.camera import CameraReader
        from acc_fusion.lidar import LidarReader

        from .distance import DistanceSmoother
        from .tracker import TrackPool

        self._can = can
        self._cam = CameraReader.from_config()
        self._lidar = LidarReader.from_config()
        self._detector = YOLODetectorAsync.from_config()
        # ACC 시나리오: 전방에 차량 항상 1 대. single_track 모드로 false positive 무시
        # + IoU 무관 재할당으로 트랙 ID 안정 + drift 방지.
        self._track_pool = TrackPool(single_track=True)
        # LiDAR raw 거리에 slew rate limit + EMA — 차량이 텔레포트하지 않는 물리 제약 강제.
        self._distance_smoother = DistanceSmoother()

        # bbox → LiDAR 매핑 파라미터 (config 의 진실 그대로).
        cam_cfg = config["camera"]
        self._img_w = int(cam_cfg["width"])
        self._hfov = float(cam_cfg["hfov"])

        # LiDAR 슬롯
        self._latest_scan = None
        self._scan_lock = threading.Lock()

        self._stop_event = threading.Event()
        self._capture_thread: threading.Thread | None = None
        self._lidar_thread: threading.Thread | None = None
        self._push_thread: threading.Thread | None = None
        self._running = False

        # show_window=True 일 때 capture worker 가 그려서 적재 (HMI 가 가져감).
        self._latest_annotated: np.ndarray | None = None
        self._annotated_lock = threading.Lock()

        # 1Hz throttled debug 로그
        self._last_update_log_time: float = 0.0

    # --------------------------------------------------------------- lifecycle
    def __enter__(self) -> "Fusion":
        if self._running:
            log.warning("Fusion already running.")
            return self
        log.info("Fusion(track-fast) 시작")
        self._cam.open()
        self._lidar.open()
        self._stop_event.clear()
        self._capture_thread = threading.Thread(
            target=self._capture_worker, daemon=True, name="tffusion-capture"
        )
        self._lidar_thread = threading.Thread(
            target=self._lidar_worker, daemon=True, name="tffusion-lidar"
        )
        self._push_thread = threading.Thread(
            target=self._push_worker, daemon=True, name="tffusion-push"
        )
        self._capture_thread.start()
        self._lidar_thread.start()
        self._push_thread.start()
        self._running = True
        log.info("Capture/LiDAR/Push 워커 시작 (Detect=NPU 콜백 + Tracker=capture 내부)")
        return self

    def __exit__(self, *_) -> None:
        if not self._running:
            return
        log.info("Fusion(track-fast) 종료 중...")
        self._stop_event.set()
        if self._capture_thread:
            self._capture_thread.join(timeout=2.0)
        if self._lidar_thread:
            self._lidar_thread.join(timeout=2.0)
        if self._push_thread:
            self._push_thread.join(timeout=2.0)
        try:
            self._detector.close()
        except Exception as e:
            log.warning(f"detector.close 실패: {e}")
        self._cam.close()
        self._lidar.close()
        self._running = False
        log.info("Fusion(track-fast) 종료 완료")

    # ----------------------------------------------------------------- workers
    def _capture_worker(self) -> None:
        last_seen_id = -1
        last_yolo_fid = -1
        while not self._stop_event.is_set():
            frame = self._cam.read()
            if frame is None:
                log.warning("Capture 워커: 프레임 읽기 실패")
                continue
            if frame.frame_id == last_seen_id:
                continue
            last_seen_id = frame.frame_id

            # 1) 비동기 YOLO dispatch (NPU 백그라운드)
            self._detector.submit(frame.image, frame.frame_id)

            # 2) 매 frame 트래커 update — 실패 시 즉시 제거
            self._track_pool.step_frame(frame.image)

            # 3) 새 YOLO 결과가 도착했으면 reconcile
            yolo_fid, yolo_dets, _ = self._detector.get_latest()
            if yolo_fid != last_yolo_fid:
                last_yolo_fid = yolo_fid
                self._track_pool.reconcile_with_yolo(yolo_dets, frame.image)

            # show_window 이면 bbox 그려서 HMI 용 슬롯에 적재
            if self._detector.show_window:
                annotated = frame.image.copy()
                self._draw_tracks(annotated, self._track_pool.snapshot())
                with self._annotated_lock:
                    self._latest_annotated = annotated

    def _lidar_worker(self) -> None:
        while not self._stop_event.is_set():
            scan = self._lidar.read()
            if scan is None:
                log.warning("LiDAR 워커: 스캔 읽기 실패")
                continue
            with self._scan_lock:
                self._latest_scan = scan

    def _push_worker(self) -> None:
        while not self._stop_event.wait(_PUSH_PERIOD_SEC):
            self._can.update_fusion_data(self.update())

    # ------------------------------------------------------------------ public
    def get_annotated_frame(self) -> np.ndarray | None:
        with self._annotated_lock:
            return None if self._latest_annotated is None else self._latest_annotated.copy()

    @property
    def show_window(self) -> bool:
        return self._detector.show_window

    def update(self) -> FusionData:
        """
        single_track 모드: 활성 트랙 1 개에 대해 bbox→LiDAR 매핑 → smoother 통과 → distance.

        return:
            FusionData:
                detected — 활성 트랙 존재 여부
                distance — smoothed LiDAR 매핑 거리 (mm). 매핑 실패 시 0.
        """
        tracks = self._track_pool.snapshot()
        if not tracks:
            # 트랙 사라짐 → smoother 도 reset (다음 트랙은 raw 부터 새로 시작)
            self._distance_smoother.reset()
            return FusionData(detected=False, distance=0)

        track = tracks[0]  # single_track 모드 — 0 또는 1 개

        with self._scan_lock:
            scan = self._latest_scan

        if scan is None or len(scan.points) == 0:
            self._throttled_debug("트랙 1개, LiDAR 데이터 없음")
            return FusionData(detected=True, distance=0)

        raw = bbox_to_distance_mm(track["bbox"], scan.points, self._hfov, self._img_w)
        if raw is None:
            self._throttled_debug("트랙 1개, bbox 영역 LiDAR 점 없음")
            return FusionData(detected=True, distance=0)

        smoothed = self._distance_smoother.step(track["id"], raw)
        self._throttled_debug(f"트랙 1개, raw={raw}mm smoothed={smoothed}mm")
        return FusionData(detected=True, distance=smoothed)

    # ----------------------------------------------------------------- helpers
    @staticmethod
    def _draw_tracks(frame: np.ndarray, tracks: list[dict]) -> None:
        import cv2

        for t in tracks:
            x1, y1, x2, y2 = t["bbox"]
            label = f"#{t['id']} {t['class_name']} {t['confidence']:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame, label, (x1, max(0, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
            )

    def _throttled_debug(self, m: str) -> None:
        now = time.monotonic()
        if now - self._last_update_log_time >= 1.0:
            log.debug(m)
            self._last_update_log_time = now
