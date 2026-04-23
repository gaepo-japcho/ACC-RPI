# ACC-RPI

ACC (Adaptive Cruise Control) 스케일카의 **Raspberry Pi 5** 측 코드.

| 모듈 | 역할 |
|---|---|
| `acc_can/` | CAN 브리지 (DBC-driven, cantools) — ECU 와 통신 |
| `acc_hmi/` | PyQt5 HMI GUI — 운전자 입력 + 상태 표시 |
| `acc_fusion/` | 카메라 + LiDAR + YOLO 센서 퓨전 — 전방 차량 감지/거리 |
| `interfaces/` | 모듈 간 공유 dataclass |
| `common/` | config/logger/singleton 공용 유틸 |

## 빠른 실행

```bash
uv sync                             # Python 3.14+ / uv 필요
uv run python acc_hmi/main.py       # GUI (시뮬레이션 모드)
```

CAN 버스 없으면 자동으로 시뮬레이션 폴백 (`[SIM] TX 0x...` 로그).

## 진실의 원천

- **CAN 계약**: `../ACC-CANDB/acc_db.dbc` (형제 repo). `acc_can/_dbc.py` 가 import 시점에 로드.
- **요구사항**: `../ACC-docs/reqs/STK/SYS/*.sdoc`
- **AUTOSAR/ECU 측**: `../ACC-autosar/`

## 상세 문서

- 디렉토리 구조/설계 원칙: [CLAUDE.md](CLAUDE.md)
- 모듈별 README: `acc_can/README.md`, `acc_fusion/README.md`

## 요구사항

- Python **3.14+** (uv 자동 관리)
- 실제 CAN 연결 시: MCP2515 또는 USB-CAN 어댑터 + `python-can` 의 socketcan 지원
