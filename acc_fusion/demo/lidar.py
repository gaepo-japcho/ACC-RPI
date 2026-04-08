from ..lidar import LidarReader
from common import get_logger

log = get_logger(__name__)


def main():
    with LidarReader.from_config() as lidar:
        log.info("LiDAR 테스트 시작 — Ctrl+C 로 종료")

        while True:
            scan = lidar.read()
            if scan is None:
                log.error("스캔 데이터를 읽을 수 없습니다.")
                break

            log.debug(
                f"scan_id={scan.scan_id} ts={scan.timestamp:.3f} "
                f"points={len(scan.points)}"
            )

            for angle, distance in scan.points:
                log.debug(f"  angle={angle:.1f}° dist={distance:.0f}mm")


if __name__ == "__main__":
    main()
