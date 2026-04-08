import cv2
from ..camera import CameraReader


def main():
    with CameraReader.from_config() as cam:
        print("카메라 테스트 시작 — 'q' 키로 종료")
        print(f"해상도: {cam.width}x{cam.height}, FPS: {cam.fps}")

        while True:
            frame = cam.read()
            if frame is None:
                print("프레임을 읽을 수 없습니다.")
                break

            print(f"frame_id={frame.frame_id} ts={frame.timestamp:.3f} shape={frame.image.shape}")

            cv2.imshow("Camera Test", frame.image)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
