import cv2
from ..camera import CameraReader
from ..yolo import YOLODetector


def main():
    detector = YOLODetector.from_config()

    with CameraReader.from_config() as cam:
        print("YOLO 감지 시작 — 'q' 키로 종료")

        while True:
            frame = cam.read()
            if frame is None:
                print("프레임을 읽을 수 없습니다.")
                break

            detections = detector.detect(frame.image)
            vis = detector.draw(frame.image, detections)

            for det in detections:
                print(
                    f"[{det['class_name']}] conf={det['confidence']:.2f} "
                    f"bbox={det['bbox']} ts={frame.timestamp:.3f}"
                )

            cv2.imshow("YOLOv26 Detection", vis)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
