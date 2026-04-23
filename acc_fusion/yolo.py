import cv2
import numpy as np
from ultralytics import YOLO
from common import get_logger

log = get_logger(__name__)


class YOLODetector:
    def __init__(self, model_path="yolo26n.pt", conf_threshold=0.5, device="cpu"):
        log.info(f"YOLO 모델 로드: {model_path} (conf={conf_threshold}, device={device})")
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        self.device = device
        log.info("YOLO 모델 로드 완료")

    def detect(self, frame: np.ndarray) -> list[dict]:
        """
        프레임에서 객체를 감지하고 결과 리스트를 반환합니다.

        Returns:
            list of dict with keys:
                - class_id (int)
                - class_name (str)
                - confidence (float)
                - bbox (tuple): (x1, y1, x2, y2)
                - center (tuple): (cx, cy)
        """
        results = self.model(
            frame, conf=self.conf_threshold, device=self.device, verbose=False
        )
        detections = []

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                class_name = self.model.names[class_id]

                detections.append(
                    {
                        "class_id": class_id,
                        "class_name": class_name,
                        "confidence": confidence,
                        "bbox": (x1, y1, x2, y2),
                        "center": ((x1 + x2) // 2, (y1 + y2) // 2),
                    }
                )

        log.debug(f"감지 결과: {len(detections)}개 객체")
        return detections

    @classmethod
    def from_config(cls) -> "YOLODetector":
        from common import config

        c = config["yolo"]
        return cls(
            model_path=c["model_path"],
            conf_threshold=c["conf_threshold"],
            device=c["device"],
        )

    def draw(self, frame: np.ndarray, detections: list[dict]) -> np.ndarray:
        """감지 결과를 프레임에 시각화합니다."""
        for det in detections:
            x1, y1, x2, y2 = det["bbox"]
            label = f"{det['class_name']} {det['confidence']:.2f}"

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame,
                label,
                (x1, y1 - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )

        return frame
