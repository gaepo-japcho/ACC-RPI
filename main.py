import acc_can as can
import acc_fusion as fusion
import acc_hmi as hmi
from common import get_logger

logger = get_logger(__name__)

def main() -> None:
    logger.info("ACC 시스템 시작")
    pass
    # TODOs
    # initialize can module
    # initialize fusion module
    # initialize and launch hmi

if __name__ == "__main__":
    main()
