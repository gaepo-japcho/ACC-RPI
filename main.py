import sys

from acc_can import CanInterface
from acc_hmi.hmi_gui import gui_main
from common import get_logger

logger = get_logger(__name__)


def main() -> int:
    logger.info("ACC 시스템 시작")
    # TODO: Fusion 통합 — `with Fusion() as fusion:` 중첩 + gui_main(can, fusion)
    with CanInterface() as can:
        return gui_main(can)


if __name__ == "__main__":
    sys.exit(main())
