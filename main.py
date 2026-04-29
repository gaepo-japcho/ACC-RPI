import sys

from acc_can import CanInterface
from acc_fusion import Fusion
from acc_hmi.hmi_gui import gui_main
from common import get_logger

logger = get_logger(__name__)


def main() -> int:
    logger.info("ACC 시스템 시작")
    with CanInterface() as can, Fusion(can):
        return gui_main(can)


if __name__ == "__main__":
    sys.exit(main())
