"""
Camera + YOLO detection view for ACC HMI.
Ref: STK002, SYS011 – 전방 차량 실시간 인식

카메라 피드에 YOLO 검출 결과(bounding box)를 오버레이하여 표시한다.
검출된 차량 중 가장 가까운(가장 큰 bbox) 차량을 전방 차량으로 판단.

YOLO 추론은 별도 QThread에서 수행하여 UI 블로킹을 방지한다.
"""
import cv2
import numpy as np
from PyQt5.QtWidgets import QLabel, QFrame, QVBoxLayout, QSizePolicy
from PyQt5.QtCore import Qt, QThread, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QImage, QPixmap, QFont


# YOLO vehicle class IDs (COCO): car=2, motorcycle=3, bus=5, truck=7
VEHICLE_CLASS_IDS = {2, 3, 5, 7}
VEHICLE_NAMES = {2: "car", 3: "motorcycle", 5: "bus", 7: "truck"}


class _InferenceThread(QThread):
    """카메라 캡처 + YOLO 추론을 별도 스레드에서 수행."""

    # Signal: (frame_rgb bytes, width, height, detections list)
    frame_ready = pyqtSignal(bytes, int, int, list)

    def __init__(self, camera_index: int, model_name: str):
        super().__init__()
        self._camera_index = camera_index
        self._model_name = model_name
        self._running = False
        self._cap = None
        self._model = None

    def run(self):
        # 카메라 초기화 (스레드 내에서)
        self._cap = cv2.VideoCapture(self._camera_index)
        if self._cap.isOpened():
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        else:
            self._cap = None

        # YOLO 모델 로드 (스레드 내에서)
        try:
            from ultralytics import YOLO
            self._model = YOLO(self._model_name)
        except Exception:
            self._model = None

        self._running = True

        while self._running:
            if self._cap is None or not self._cap.isOpened():
                # 카메라 없음 – placeholder
                self._emit_no_camera()
                self.msleep(200)
                continue

            ret, frame = self._cap.read()
            if not ret:
                self._emit_no_camera()
                self.msleep(100)
                continue

            # YOLO 추론 (여기서 시간이 걸려도 UI는 안 멈춤)
            detections = []
            if self._model is not None:
                results = self._model(frame, verbose=False, conf=0.4)
                detections = self._process_results(frame, results)

            # BGR → RGB 변환 후 signal 전송
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, _ = rgb.shape
            self.frame_ready.emit(rgb.tobytes(), w, h, detections)

        # 정리
        if self._cap is not None:
            self._cap.release()

    def _process_results(self, frame: np.ndarray, results) -> list[dict]:
        """YOLO 결과 → 차량 필터링 + bbox 그리기."""
        detections = []
        for r in results:
            boxes = r.boxes
            if boxes is None:
                continue
            for box in boxes:
                cls_id = int(box.cls[0])
                if cls_id not in VEHICLE_CLASS_IDS:
                    continue

                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                area = (x2 - x1) * (y2 - y1)
                label = VEHICLE_NAMES.get(cls_id, "vehicle")

                detections.append({
                    "cls": cls_id, "conf": conf,
                    "bbox": (x1, y1, x2, y2), "area": area,
                    "label": label,
                })

                # Draw bbox
                color = (0, 200, 255)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                text = f"{label} {conf:.0%}"
                (tw, th), _ = cv2.getTextSize(
                    text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1
                )
                cv2.rectangle(
                    frame, (x1, y1 - th - 8), (x1 + tw + 4, y1), color, -1
                )
                cv2.putText(
                    frame, text, (x1 + 2, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1,
                )

        # 가장 가까운 차량(가장 큰 bbox) 하이라이트
        if detections:
            nearest = max(detections, key=lambda d: d["area"])
            x1, y1, x2, y2 = nearest["bbox"]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
            cv2.putText(
                frame, "TRACKING", (x1, y2 + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
            )

        return detections

    def _emit_no_camera(self):
        w, h = 640, 480
        img = np.zeros((h, w, 3), dtype=np.uint8)
        cv2.putText(
            img, "NO CAMERA", (w // 2 - 100, h // 2 - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (80, 80, 80), 2,
        )
        cv2.putText(
            img, "Connect camera to enable",
            (w // 2 - 140, h // 2 + 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (60, 60, 60), 1,
        )
        self.frame_ready.emit(img.tobytes(), w, h, [])

    def stop(self):
        self._running = False
        self.wait(2000)


class CameraView(QFrame):
    """카메라 + YOLO 인식 결과 표시 위젯."""

    def __init__(self, camera_index: int = 0, model_name: str = "yolo26n.pt",
                 parent=None):
        super().__init__(parent)
        self.setStyleSheet("background-color: #000; border: none;")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(4)

        # Video display
        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background: #000; border: none;")
        self.video_label.setSizePolicy(
            QSizePolicy.Expanding, QSizePolicy.Expanding
        )
        self.video_label.setMinimumSize(320, 240)
        layout.addWidget(self.video_label)

        # Detection info
        self.info_label = QLabel("카메라 초기화 중...")
        self.info_label.setAlignment(Qt.AlignCenter)
        self.info_label.setFont(QFont("Consolas", 11))
        self.info_label.setStyleSheet("color: #aaa; background: transparent;")
        layout.addWidget(self.info_label)

        # Callback
        self._on_vehicle_detected = None

        # 별도 스레드에서 카메라 + YOLO 실행
        self._thread = _InferenceThread(camera_index, model_name)
        self._thread.frame_ready.connect(self._on_frame_ready)
        self._thread.start()

    def set_vehicle_callback(self, fn):
        """fn(detected: bool, bbox_area: int) – 전방 차량 감지 콜백."""
        self._on_vehicle_detected = fn

    @pyqtSlot(bytes, int, int, list)
    def _on_frame_ready(self, rgb_bytes: bytes, w: int, h: int,
                        detections: list):
        """추론 스레드에서 프레임이 도착하면 UI 갱신 (메인 스레드)."""
        # QImage 생성
        qimg = QImage(rgb_bytes, w, h, 3 * w, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)

        # 위젯 크기에 맞춰 스케일
        label_size = self.video_label.size()
        scaled = pixmap.scaled(
            label_size, Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        self.video_label.setPixmap(scaled)

        # 감지 정보 업데이트
        vehicle_count = len(detections)
        if vehicle_count > 0:
            nearest = max(detections, key=lambda d: d["area"])
            self.info_label.setText(
                f"차량 {vehicle_count}대 감지 | "
                f"최근접: {nearest['label']} (conf: {nearest['conf']:.0%})"
            )
            self.info_label.setStyleSheet(
                "color: #ff9800; background: transparent;"
            )
            if self._on_vehicle_detected:
                self._on_vehicle_detected(True, nearest["area"])
        else:
            self.info_label.setText("전방 차량 미감지")
            self.info_label.setStyleSheet(
                "color: #aaa; background: transparent;"
            )
            if self._on_vehicle_detected:
                self._on_vehicle_detected(False, 0)

    def stop(self):
        self._thread.stop()

    def closeEvent(self, event):
        self.stop()
        super().closeEvent(event)
