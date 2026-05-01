"""
acc_track_fast_fusion 동작 확인용 카메라 데모.

acc_fast_fusion/demo/yolo_cam.py 와 차이:
- 매 frame 트래커 update + YOLO 갱신 시 reconcile.
- bbox 라벨에 트랙 ID + LiDAR 매핑 거리(m) 표시.
- HUD 에 트랙 수 / 마지막 YOLO 갱신 lag 표시.
- LiDAR 는 별도 thread 백그라운드 read (5~10Hz), 메인 루프는 latest scan polling.

실행:
    uv run python -m acc_track_fast_fusion.demo.yolo_cam

키:
    q  종료
"""
import threading
import time
from collections import deque

import cv2

from acc_fast_fusion.yolo import YOLODetectorAsync
from acc_fusion.camera import CameraReader
from acc_fusion.lidar import LidarReader
from common import config, get_logger

from .. import bbox_to_distance_mm
from ..distance import DistanceSmoother
from ..tracker import TrackPool

log = get_logger(__name__)


class _LidarSlot:
    """LiDAR 백그라운드 thread 가 latest scan 만 적재하는 슬롯."""

    def __init__(self):
        self._scan = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lidar: LidarReader | None = None

    def start(self) -> None:
        self._lidar = LidarReader.from_config()
        self._lidar.open()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="demo-lidar")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._lidar:
            self._lidar.close()

    def _loop(self) -> None:
        while not self._stop.is_set():
            scan = self._lidar.read()
            if scan is None:
                continue
            with self._lock:
                self._scan = scan

    def latest(self):
        with self._lock:
            return self._scan


def _draw_overlay(
    frame,
    tracks_with_distance: list[tuple],
    cap_fps: float,
    yolo_fps: float,
    yolo_lag_frames: int,
) -> None:
    for t, raw_mm, smoothed_mm in tracks_with_distance:
        x1, y1, x2, y2 = t["bbox"]
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        # 라벨 두 줄: ID/클래스/conf, smoothed 거리(+raw)
        line1 = f"#{t['id']} {t['class_name']} {t['confidence']:.2f}"
        if smoothed_mm is not None:
            line2 = f"{smoothed_mm/1000:.2f}m  (raw {raw_mm/1000:.2f}m)"
            color = (0, 255, 255)
        else:
            line2 = "-- m"
            color = (128, 128, 128)
        cv2.putText(
            frame, line1, (x1, max(0, y1 - 24)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
        )
        cv2.putText(
            frame, line2, (x1, max(0, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2,
        )

    nearest = min(
        (s for _, _, s in tracks_with_distance if s is not None), default=None
    )
    hud = [
        f"capture: {cap_fps:5.1f} fps",
        f"yolo   : {yolo_fps:5.1f} fps",
        (f"yolo lag: {yolo_lag_frames:>3d} frames"
         if yolo_lag_frames >= 0 else "yolo lag: --- (warming up)"),
        f"tracks : {len(tracks_with_distance)}",
        (f"nearest: {nearest/1000:.2f}m"
         if nearest is not None else "nearest: -- (LiDAR 매핑 없음)"),
    ]
    for i, line in enumerate(hud):
        y = 20 + i * 22
        cv2.putText(frame, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 255), 1, cv2.LINE_AA)


def _fps_from_window(times: deque) -> float:
    if len(times) < 2:
        return 0.0
    return (len(times) - 1) / max(1e-6, times[-1] - times[0])


def main():
    detector = YOLODetectorAsync.from_config()
    pool = TrackPool(single_track=True)  # ACC 시나리오 — 차량 1 대
    smoother = DistanceSmoother()
    lidar_slot = _LidarSlot()

    img_w = int(config["camera"]["width"])
    hfov = float(config["camera"]["hfov"])

    cap_times: deque = deque(maxlen=60)
    yolo_times: deque = deque(maxlen=60)
    last_yolo_fid = -1

    try:
        lidar_slot.start()
        with CameraReader.from_config() as cam:
            log.info("YOLO(Async)+Tracker+LiDAR 데모 시작 — 'q' 키로 종료")
            log.info(f"카메라: {cam.width}x{cam.height} @ {cam.fps}fps, hfov={hfov}°")

            while True:
                frame = cam.read()
                if frame is None:
                    log.error("프레임을 읽을 수 없습니다.")
                    break

                detector.submit(frame.image, frame.frame_id)
                cap_times.append(time.monotonic())

                pool.step_frame(frame.image)

                yolo_fid, yolo_dets, _ = detector.get_latest()
                if yolo_fid != last_yolo_fid:
                    last_yolo_fid = yolo_fid
                    yolo_times.append(time.monotonic())
                    pool.reconcile_with_yolo(yolo_dets, frame.image)

                tracks = pool.snapshot()

                # 각 트랙 bbox → LiDAR 거리 매핑 + smoothing.
                # 트랙 0 이면 smoother reset (다음 트랙은 raw 부터 새로 시작).
                scan = lidar_slot.latest()
                if not tracks:
                    smoother.reset()
                tracks_with_distance: list[tuple] = []
                for t in tracks:
                    raw = (bbox_to_distance_mm(t["bbox"], scan.points, hfov, img_w)
                           if scan is not None else None)
                    smoothed = smoother.step(t["id"], raw) if raw is not None else None
                    tracks_with_distance.append((t, raw, smoothed))

                cap_fps = _fps_from_window(cap_times)
                yolo_fps = _fps_from_window(yolo_times)
                yolo_lag = (
                    max(0, frame.frame_id - last_yolo_fid)
                    if last_yolo_fid >= 0 else -1
                )

                vis = frame.image.copy()
                _draw_overlay(vis, tracks_with_distance, cap_fps, yolo_fps, yolo_lag)
                cv2.imshow("acc_track_fast_fusion - YOLO+Tracker+LiDAR", vis)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        detector.close()
        lidar_slot.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
