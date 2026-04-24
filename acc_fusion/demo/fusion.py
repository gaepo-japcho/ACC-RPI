import time
from acc_can import CanInterface
from .. import Fusion
from common import get_logger

log = get_logger(__name__)


def main():
    with CanInterface() as can, Fusion(can) as fusion:
        log.info("Fusion 테스트 시작 — Ctrl+C 로 종료")

        while True:
            data = fusion.update()
            log.debug(f"detected={data.detected} distance={data.distance}mm")
            time.sleep(0.02)  # 20ms (SENSOR_FUSION TX 주기와 동일)


if __name__ == "__main__":
    main()
