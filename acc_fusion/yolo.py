import cv2
import numpy as np
from hailo_platform import (
    HEF,
    VDevice,
    FormatType,
    HailoStreamInterface,
    ConfigureParams,
    InputVStreamParams,
    OutputVStreamParams,
    InferVStreams,
)

from common import get_logger

log = get_logger(__name__)


# COCO 80 클래스 (yolov6n_h8l.hef 학습 데이터셋)
COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]

# 차량 관련 클래스만 필터 (COCO 기준)
VEHICLE_CLASSES = {"car", "truck", "bus", "motorcycle", "bicycle"}


class YOLODetector:
    def __init__(
        self,
        model_path: str = "/usr/share/hailo-models/yolov6n_h8l.hef",
        conf_threshold: float = 0.5,
        device: str = "npu",
    ):
        log.info(f"YOLO NPU 모델 로드: {model_path} (conf={conf_threshold})")
        if device != "npu":
            log.warning(f"device='{device}' 무시 — NPU 고정")

        self.conf_threshold = conf_threshold
        self.device = "npu"

        self._hef = HEF(model_path)
        self._vdevice = VDevice()
        configure_params = ConfigureParams.create_from_hef(
            hef=self._hef, interface=HailoStreamInterface.PCIe
        )
        self._network_group = self._vdevice.configure(self._hef, configure_params)[0]
        self._network_group_params = self._network_group.create_params()

        self._input_params = InputVStreamParams.make(
            self._network_group, format_type=FormatType.UINT8
        )
        self._output_params = OutputVStreamParams.make(
            self._network_group, format_type=FormatType.FLOAT32
        )

        input_info = self._hef.get_input_vstream_infos()[0]
        self._input_name = input_info.name
        self._input_h, self._input_w, _ = input_info.shape
        self.names = {i: n for i, n in enumerate(COCO_NAMES)}

        # 인스턴스 수명 동안 활성화 유지 — close() 에서 해제
        self._activation = self._network_group.activate(self._network_group_params)
        self._activation.__enter__()

        log.info(
            f"YOLO NPU 모델 로드 완료 (input={self._input_w}x{self._input_h})"
        )

    def detect(self, frame: np.ndarray) -> list[dict]:
        """
        프레임에서 차량 객체를 감지하고 결과 리스트를 반환합니다.

        Args:
            frame: RGB 또는 BGR numpy 배열 (H, W, 3).
                Picamera2 의 RGB888 또는 cv2 의 BGR 모두 가능.

        Returns:
            list of dict with keys:
                - class_id (int)
                - class_name (str)
                - confidence (float)
                - bbox (tuple): (x1, y1, x2, y2) — 원본 frame 좌표
                - center (tuple): (cx, cy)
        """
        orig_h, orig_w = frame.shape[:2]

        input_tensor, scale, pad_x, pad_y = self._letterbox(
            frame, self._input_w, self._input_h
        )
        input_data = {self._input_name: np.expand_dims(input_tensor, axis=0)}

        with InferVStreams(
            self._network_group, self._input_params, self._output_params
        ) as infer_pipeline:
            results = infer_pipeline.infer(input_data)

        # 출력: list[batch=1] of list[80 classes] of np.ndarray (N, 5).
        # 각 행: [y_min, x_min, y_max, x_max, score] (정규화 0~1)
        nms_out = next(iter(results.values()))[0]

        detections = []

        for class_id, class_dets in enumerate(nms_out):
            class_name = (
                COCO_NAMES[class_id] if class_id < len(COCO_NAMES) else str(class_id)
            )
            if class_name not in VEHICLE_CLASSES:
                continue
            if len(class_dets) == 0:
                continue

            for det_row in class_dets:
                y_min, x_min, y_max, x_max, score = det_row
                if score < self.conf_threshold:
                    continue

                # 정규화 좌표(0~1) → letterbox 공간 → 원본 좌표
                x1 = int((x_min * self._input_w - pad_x) / scale)
                y1 = int((y_min * self._input_h - pad_y) / scale)
                x2 = int((x_max * self._input_w - pad_x) / scale)
                y2 = int((y_max * self._input_h - pad_y) / scale)

                x1 = max(0, min(orig_w - 1, x1))
                y1 = max(0, min(orig_h - 1, y1))
                x2 = max(0, min(orig_w - 1, x2))
                y2 = max(0, min(orig_h - 1, y2))

                if x2 <= x1 or y2 <= y1:
                    continue

                detections.append(
                    {
                        "class_id": class_id,
                        "class_name": class_name,
                        "confidence": float(score),
                        "bbox": (x1, y1, x2, y2),
                        "center": ((x1 + x2) // 2, (y1 + y2) // 2),
                    }
                )

        log.debug(f"감지 결과: {len(detections)}개 객체")
        return detections

    @staticmethod
    def _letterbox(
        img: np.ndarray, target_w: int, target_h: int, fill: int = 114
    ) -> tuple[np.ndarray, float, int, int]:
        """비율 유지하며 target 크기로 패딩 리사이즈."""
        h, w = img.shape[:2]
        scale = min(target_w / w, target_h / h)
        new_w, new_h = int(round(w * scale)), int(round(h * scale))
        resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        pad_x = (target_w - new_w) // 2
        pad_y = (target_h - new_h) // 2
        canvas = np.full((target_h, target_w, 3), fill, dtype=np.uint8)
        canvas[pad_y : pad_y + new_h, pad_x : pad_x + new_w] = resized
        return canvas, scale, pad_x, pad_y

    def close(self):
        """NPU 리소스 해제."""
        try:
            if self._activation is not None:
                self._activation.__exit__(None, None, None)
                self._activation = None
        except Exception as e:
            log.warning(f"NPU activation 해제 실패: {e}")
        try:
            self._vdevice.release()
        except Exception:
            pass
        log.info("YOLO NPU 모델 해제")

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    @classmethod
    def from_config(cls) -> "YOLODetector":
        from common import config

        c = config.config["yolo"] if hasattr(config, "config") else config["yolo"]
        return cls(
            model_path=c["model_path"],
            conf_threshold=c["conf_threshold"],
            device=c.get("device", "npu"),
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
