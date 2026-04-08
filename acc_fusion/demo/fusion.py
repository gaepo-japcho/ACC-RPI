import time
from .. import Fusion


def main():
    with Fusion() as fusion:
        print("Fusion 테스트 시작 — Ctrl+C 로 종료")

        while True:
            data = fusion.update()
            print(f"detected={data.detected} distance={data.distance}cm")
            time.sleep(0.02)  # 20ms (CAN TX 주기와 동일)


if __name__ == "__main__":
    main()
