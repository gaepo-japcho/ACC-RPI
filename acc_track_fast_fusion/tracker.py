"""
OpenCV CSRT 기반 차량 트랙 풀 (opencv-contrib-python 필요).

정책 (acc_track_fast_fusion 의 핵심):
  1. 트래커 invalidate 는 bbox 가 카메라 화각 밖으로 벗어날 때 (가시 영역 < 10%) + CSRT
     자체 fail 일 때.
  2. CSRT 가 자체적으로 scale estimator (DSST-like) + HOG/Color Names feature 로 차량의
     거리 변화/외관 변화를 추적. 자체 multi-scale brute force 가 아니라 학습 기반.
  3. YOLO 갱신 도착 시 IoU 매칭 — 매칭 안 된 트랙은 즉시 제거. "차량이 화면에서
     사라지면 즉시 bbox 제거" 의 진짜 보장 layer.
"""
import itertools
import threading

import cv2
import numpy as np

from common import get_logger

log = get_logger(__name__)


# 트랙↔YOLO 매칭 시 IoU 하한. 비동기라 YOLO 와 현재 frame 사이 lag 가 있으니 약간 느슨하게.
_IOU_MATCH_THRESHOLD = 0.2

# bbox 의 화면 안 가시 영역이 이 비율 미만이면 fail (= 화각 밖으로 거의 다 빠짐).
_VISIBLE_AREA_RATIO = 0.1

# YOLO reconcile 에서 매칭 실패가 이 횟수 누적되면 트랙 제거 (grace).
# 한 번 잡힌 차량이 갑자기 사라질 일은 거의 없고, YOLO 가 일시적으로 못 잡는 경우 (모션
# 블러, 부분 가림, conf 떨어짐) 가 있으니 CSRT 가 위치 보간으로 메꿈.
# YOLO ~20-30Hz 기준 5 사이클 ≈ 150-250ms grace. 너무 길면 사라진 차량 bbox 가 오래 남고,
# 너무 짧으면 jitter 에 트랙 끊김. 5 가 합리적 시작점.
# 트래커 자체 fail (화각 밖, 가시영역 < 10%) 은 grace 무시하고 즉시 제거 (step_frame).
_MAX_MISSED_YOLO_RECONCILES = 5

_id_counter = itertools.count(1)


def _iou(a: tuple, b: tuple) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    if inter == 0:
        return 0.0
    a_area = max(1, (ax2 - ax1) * (ay2 - ay1))
    b_area = max(1, (bx2 - bx1) * (by2 - by1))
    return inter / (a_area + b_area - inter)


class _BboxTracker:
    """
    cv2.TrackerCSRT wrapper.

    CSRT (Channel and Spatial Reliability Tracker) — HOG + Color Names + grayscale 멀티
    채널 + DSST-style scale estimator + spatial reliability map. ACC 시나리오에서 차량
    거리 변화로 인한 bbox 스케일 변화를 자체적으로 추적.

    OpenCV 트래커 인터페이스 차이:
      - bbox 는 (x, y, w, h) — 우리는 (x1, y1, x2, y2) 로 노출하므로 양쪽 변환.
      - update() 가 score 안 줌. fail 은 단지 boolean.
    """

    def __init__(self):
        self._tracker = cv2.TrackerCSRT_create()
        self._bbox: tuple[int, int, int, int] | None = None  # (x1, y1, x2, y2)

    def init(self, frame: np.ndarray, bbox: tuple) -> bool:
        h_img, w_img = frame.shape[:2]
        x1, y1, x2, y2 = (int(v) for v in bbox)
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w_img, x2), min(h_img, y2)
        w, h = x2 - x1, y2 - y1
        if w < 8 or h < 8:
            return False
        try:
            self._tracker.init(frame, (x1, y1, w, h))
        except cv2.error as e:
            log.warning(f"CSRT init 실패: {e}")
            return False
        self._bbox = (x1, y1, x2, y2)
        return True

    def update(self, frame: np.ndarray) -> tuple[bool, tuple | None]:
        """
        반환 (alive, bbox):
          alive=False — 화각 밖으로 거의 다 빠졌거나 CSRT 가 자체 fail.
          alive=True  — 트랙 살아있음. bbox 갱신.
        """
        if self._bbox is None:
            return False, None
        h_img, w_img = frame.shape[:2]
        x1, y1, x2, y2 = self._bbox
        bw, bh = x2 - x1, y2 - y1

        # invalidate 조건 1: bbox 가시 영역이 너무 작음 (= 거의 다 화각 밖).
        ix1, iy1 = max(x1, 0), max(y1, 0)
        ix2, iy2 = min(x2, w_img), min(y2, h_img)
        if max(0, ix2 - ix1) * max(0, iy2 - iy1) < bw * bh * _VISIBLE_AREA_RATIO:
            return False, None

        try:
            ok, xywh = self._tracker.update(frame)
        except cv2.error as e:
            log.warning(f"CSRT update 실패: {e}")
            return False, None

        # invalidate 조건 2: CSRT 자체 fail.
        if not ok:
            return False, None

        x, y, w, h = xywh
        nx1, ny1 = int(x), int(y)
        nx2, ny2 = int(x + w), int(y + h)
        # 화면 안으로 clamp (CSRT 가 가끔 음수/초과 좌표 반환)
        nx1 = max(0, min(w_img - 1, nx1))
        ny1 = max(0, min(h_img - 1, ny1))
        nx2 = max(nx1 + 8, min(w_img, nx2))
        ny2 = max(ny1 + 8, min(h_img, ny2))
        self._bbox = (nx1, ny1, nx2, ny2)
        return True, self._bbox


class Track:
    __slots__ = ("id", "class_id", "class_name", "bbox", "confidence",
                 "missed_yolo", "_tracker")

    def __init__(self, bbox: tuple, class_id: int, class_name: str,
                 confidence: float, frame: np.ndarray):
        self.id = next(_id_counter)
        self.class_id = class_id
        self.class_name = class_name
        self.bbox = tuple(int(v) for v in bbox)
        self.confidence = float(confidence)
        # 연속 YOLO 매칭 실패 카운터. _MAX_MISSED_YOLO_RECONCILES 도달 시 제거.
        self.missed_yolo = 0
        self._tracker = _BboxTracker()
        self._tracker.init(frame, self.bbox)

    def update_from_tracker(self, frame: np.ndarray) -> bool:
        ok, new_bbox = self._tracker.update(frame)
        if not ok:
            return False
        self.bbox = new_bbox
        return True

    def reseed_from_yolo(self, det: dict, frame: np.ndarray) -> None:
        # YOLO 가 진실 — 매 reconcile 마다 새 CSRT 로 reset. CSRT 의 학습 상태가
        # stale 해서 drift 하는 위험을 제거 (fps 편차는 trade-off).
        self.bbox = tuple(int(v) for v in det["bbox"])
        self.confidence = float(det["confidence"])
        self.missed_yolo = 0  # 매칭 성공 → 카운터 리셋
        self._tracker = _BboxTracker()
        self._tracker.init(frame, self.bbox)

    def snapshot(self) -> dict:
        x1, y1, x2, y2 = self.bbox
        return {
            "id": self.id,
            "class_id": self.class_id,
            "class_name": self.class_name,
            "confidence": self.confidence,
            "bbox": self.bbox,
            "center": ((x1 + x2) // 2, (y1 + y2) // 2),
        }


class TrackPool:
    """
    single_track=True (ACC 시나리오 — 차량 항상 1 대):
      - YOLO 검출 중 confidence 가장 높은 1 개만 채택 (false positive 자동 무시).
      - 트랙은 항상 0 또는 1 개. 살아있는 트랙이 있으면 IoU 무관 재할당 → ID 안정성.
      - 트랙이 0 일 때만 신규 생성.

    single_track=False (일반):
      - IoU 매칭 → reseed / 신규 / grace counter.
    """

    def __init__(self, single_track: bool = False):
        self._tracks: list[Track] = []
        self._lock = threading.Lock()
        self._single_track = single_track

    def step_frame(self, frame: np.ndarray) -> None:
        """매 카메라 frame 호출. CSRT update 실패 트랙 즉시 제거."""
        with self._lock:
            self._tracks = [t for t in self._tracks if t.update_from_tracker(frame)]

    def reconcile_with_yolo(self, detections: list[dict], frame: np.ndarray) -> None:
        """YOLO 갱신 시 호출. 모드별로 분기."""
        if self._single_track:
            self._reconcile_single(detections, frame)
        else:
            self._reconcile_multi(detections, frame)

    def _reconcile_single(self, detections: list[dict], frame: np.ndarray) -> None:
        """차량 1 대 시나리오 — 가장 confident 한 검출만 사용."""
        with self._lock:
            best_det = max(detections, key=lambda d: d["confidence"], default=None)

            if best_det is None:
                # YOLO 가 아무것도 못 잡음 → grace counter 증가
                survivors = []
                for t in self._tracks:
                    t.missed_yolo += 1
                    if t.missed_yolo < _MAX_MISSED_YOLO_RECONCILES:
                        survivors.append(t)
                self._tracks = survivors
                return

            if self._tracks:
                # 살아있는 트랙이 있으면 IoU 무관 재할당 (ID 유지, drift 방지)
                self._tracks[0].reseed_from_yolo(best_det, frame)
            else:
                # 트랙 없음 → 신규 생성
                self._tracks = [
                    Track(best_det["bbox"], best_det["class_id"], best_det["class_name"],
                          best_det["confidence"], frame)
                ]

    def _reconcile_multi(self, detections: list[dict], frame: np.ndarray) -> None:
        """일반 다중 트랙 — IoU 매칭."""
        with self._lock:
            unmatched = list(self._tracks)
            kept: list[Track] = []
            for det in detections:
                best, best_iou = None, _IOU_MATCH_THRESHOLD
                for t in unmatched:
                    if t.class_id != det["class_id"]:
                        continue
                    iou = _iou(t.bbox, det["bbox"])
                    if iou > best_iou:
                        best, best_iou = t, iou
                if best is not None:
                    best.reseed_from_yolo(det, frame)
                    kept.append(best)
                    unmatched.remove(best)
                else:
                    kept.append(
                        Track(det["bbox"], det["class_id"], det["class_name"],
                              det["confidence"], frame)
                    )
            for t in unmatched:
                t.missed_yolo += 1
                if t.missed_yolo < _MAX_MISSED_YOLO_RECONCILES:
                    kept.append(t)
            self._tracks = kept

    def snapshot(self) -> list[dict]:
        with self._lock:
            return [t.snapshot() for t in self._tracks]

    def clear(self) -> None:
        with self._lock:
            self._tracks.clear()
