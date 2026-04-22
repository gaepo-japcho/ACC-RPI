"""
ACC HMI 단독 실행 엔트리.

    python -m acc_hmi.main

프로세스 내 `acc_can.CanInterface` 를 시작하고, 동일 인스턴스를 GUI 에 주입한다.
`acc_can` 은 `python-can` 부재 / CAN 버스 open 실패 시 자동으로 simulation 모드로
동작하므로 하드웨어가 없어도 GUI 는 뜬다.

전체 시스템 통합 실행은 루트 `main.py` 를 사용한다.
"""
import sys

from acc_can import CanInterface
from acc_hmi.hmi_gui import gui_main


def main() -> int:
    can = CanInterface()
    can.start()
    try:
        return gui_main(can)
    finally:
        can.stop()


if __name__ == "__main__":
    sys.exit(main())
