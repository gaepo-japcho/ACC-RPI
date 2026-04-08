from ..lidar import LidarReader


def main():
    with LidarReader.from_config() as lidar:
        print("LiDAR 테스트 시작 — Ctrl+C 로 종료")

        while True:
            scan = lidar.read()
            if scan is None:
                print("스캔 데이터를 읽을 수 없습니다.")
                break

            print(
                f"scan_id={scan.scan_id} ts={scan.timestamp:.3f} "
                f"points={len(scan.points)}"
            )

            for angle, distance in scan.points:
                print(f"  angle={angle:.1f}° dist={distance:.0f}mm")


if __name__ == "__main__":
    main()
