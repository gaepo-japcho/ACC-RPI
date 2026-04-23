# CLAUDE.md — ACC-RPI

이 저장소는 ACC 스케일카의 **Raspberry Pi 5** 측 코드다. HMI (PyQt5 GUI), 카메라 + LiDAR 센서 퓨전, CAN 브리지를 담당한다. 독립 git repo. 상위 맥락은 `/mnt/c/acc/CLAUDE.md` 참조.

## 디렉토리 구조

```
ACC-RPI/
├── main.py                     # ★ 진짜 진입점 아님 — 모듈 import 만 하는 스텁
├── pyproject.toml              # uv 기반 의존성 (Python >= 3.14)
├── uv.lock
├── .python-version             # "3.14"
├── config.toml                 # camera/lidar/yolo 하드웨어 파라미터
├── README.md
│
├── acc_can/                    # CAN 브리지 모듈 (DBC-driven, cantools 기반)
│   ├── __init__.py             # CanInterface 클래스 — 스레딩·상태·공개 API
│   ├── _dbc.py                 # DBC 로드 + MSG_ID_* 상수 + msg()/msg_id() 헬퍼
│   └── _codec.py               # encode/decode pure function 6 개 (TX 4 / RX 2)
│
├── acc_hmi/                    # PyQt5 HMI (실제 진입점)
│   ├── main.py                 # hmi_gui.gui_main() 호출
│   ├── hmi_gui.py              # PyQt5 GUI (~780 줄, 커스텀 게이지/페달/버튼)
│   └── acc_state.py            # AccStatus enum + UI 상수 + 버튼 가용성 테이블
│
├── acc_fusion/                 # 카메라 + LiDAR + YOLO 센서 퓨전
│   ├── __init__.py             # Fusion 클래스 — 센서 orchestrator
│   ├── camera.py               # CameraFrame + CameraReader (싱글톤)
│   ├── lidar.py                # LidarScan + LidarReader (싱글톤)
│   ├── yolo.py                 # YOLODetector (차량 감지)
│   └── demo/                   # 센서 개별 테스트 스크립트
│
├── interfaces/                 # 모듈 간 공유 dataclass (dumb 데이터 그릇)
│   ├── acc_info.py             # AccInfo (RX: ACC 상태 피드백)
│   ├── acc_setting.py          # AccSetting (TX: 목표 속도/레벨)
│   ├── acc_status.py           # AccStatus Enum (OFF/STANDBY/CRUISING/FOLLOWING/FAULT)
│   ├── button_input.py         # ButtonInput (TX: ACC 버튼 edge trigger)
│   ├── ecu_status.py           # EcuStatus (RX: HB/ERR)
│   ├── fusion_data.py          # FusionData (전방 차량 감지/거리 mm)
│   ├── fusion.py               # Fusion 관련 타입
│   ├── pedal_input.py          # PedalInput (TX: brake/accel_pwm)
│   └── vehicle_info.py         # VehicleInfo (RX: 자차 속도)
│
└── common/                     # 공용 유틸
    ├── config.py               # config.toml 로더
    ├── logger.py               # logging 헬퍼
    └── singleton.py            # Singleton metaclass
```

## 진입점

루트 `main.py` 는 스텁. 실제 GUI 는:

```bash
uv sync                         # cantools + python-can + PyQt 등 전부 설치
uv run python acc_hmi/main.py   # 시뮬레이션 모드로 GUI 실행
                                # (CAN 버스 없으면 _send_raw 가 [SIM] 로그만 찍음)
```

## 진실의 원천 — CAN 계약은 DBC

**모든 CAN ID / 페이로드 레이아웃 / 시그널 범위는 `../ACC-CANDB/acc_db.dbc` 가 결정한다.** Python 코드는 하드코딩하지 않는다.

`acc_can/_dbc.py` 가 import 시점에 DBC 를 로드하고 `MSG_ID_*` 상수를 DBC 에서 조회한다. `_codec.py` 의 encode/decode 는 `cantools.database.Message.encode(signals_dict)` 에 비트/바이트 레이아웃을 전부 위임한다.

**DBC 가 바뀌면 Python 코드는 원칙적으로 안 고쳐도 된다** (시그널 이름·dataclass 매핑이 달라지는 경우만 예외).

### CAN 메시지 (DBC 2026-04-23 기준)

| ID | 이름 | 방향 | 주기 | Python 처리 |
|---|---|---|---|---|
| `0x110` | `SENSOR_FUSION` | RPi → ECU | 20ms | `_tx_sensor_fusion` |
| `0x111` | `SENSOR_HEARTBEAT` | RPi → ECU | 20ms | `_tx_sensor_heartbeat` |
| `0x120` | `VEH_CTRL` | RPi → ECU | 20ms | `_tx_veh_ctrl` (brake+accel_pwm) |
| `0x510` | `ACC_CTRL` | RPi → ECU | 50ms | `_tx_acc_ctrl` (버튼 edge+목표 속도/레벨) |
| `0x410` | `ECU_HEARTBEAT` | ECU → RPi | 10ms | `_parse_ecu_heartbeat` |
| `0x520` | `ACC_STATUS` | ECU → RPi | 50ms | `_parse_acc_status` |

**미구현**: `0x300 MTR_SPD_FB` (자차속도) RX. 현재 `VehicleInfo.current_speed` 는 0.0 고정. HMI 자차속도 표시 필요 시 추가.

## acc_can 구조 (2026-04-23 리팩터)

3 모듈 분리로 관심사 격리:

- **`_dbc.py`** — DBC 접근의 단일 진입점. 다른 모듈 (acc_fusion 등) 도 `from acc_can._dbc import msg, msg_id, MSG_ID_*` 로 재활용 가능. DBC 로드 실패 시 ImportError 전파 (엄격).
- **`_codec.py`** — pure function encoders/decoders. 스레드·락·버스 의존 0 → 단위 테스트 자유.
  - TX: `encode_veh_ctrl / encode_acc_ctrl / encode_sensor_fusion / encode_sensor_heartbeat`
  - RX: `decode_acc_status / decode_ecu_heartbeat` (둘 다 `decode_choices=False` 로 NamedSignalValue 회피 — raw int 반환)
- **`__init__.py`** — `CanInterface` 클래스. 스레딩 (TX/RX/HB 3 스레드), 상태 버퍼 (lock 보호), 공개 API (`send_*`, `get_*`, `start/stop/is_connected`).

### TX 주기 (multi-rate)

- **20ms**: `VEH_CTRL`, `SENSOR_FUSION`, `SENSOR_HEARTBEAT`
- **50ms**: `ACC_CTRL`
- `_tx_loop` 은 5ms 폴링으로 두 deadline 을 독립 관리.

### 단위 / 범위 (DBC 정합)

| 필드 | 타입 | 범위 | 단위 | DBC 시그널 |
|---|---|---|---|---|
| `PedalInput.accel_pwm` | int | -128~127 | — | `SET_ACCEL_PWM` int8 |
| `PedalInput.brake` | bool | 0/1 | — | `BTN_BRAKE` bit |
| `AccSetting.set_speed` | int | -500~500 | cm/s | `SET_ACC_SPD` int16 |
| `AccSetting.distance_level` | int | 1~3 | — | `SET_ACC_LVL` uint8 |
| `FusionData.distance` | int | 0~12000 | **mm** | `VEH_DIST` uint16 |

## acc_fusion 구조

- **`camera.py`** — `CameraReader` (Singleton). `config.toml [camera]` 의 source/width/height/fps 로드. `read()` → `CameraFrame`.
- **`lidar.py`** — `LidarReader` (Singleton, `rplidar` 사용). `config.toml [lidar]` 의 port/baudrate/angle_offset. `scan()` → `LidarScan`.
- **`yolo.py`** — `YOLODetector`. `config.toml [yolo]` 의 model_path/conf_threshold. YOLO v26 pretrained (ultralytics).
- **`__init__.py`** — `Fusion` 클래스 (Singleton). 세 센서를 조합해 `FusionData` 산출. 이를 `CanInterface.update_fusion_data()` 로 acc_can 에 전달.

**MRR-30 LiDAR 스펙**: `../ACC-docs/refs/20220901 MRR-30 User Manual & Cert List_00.pdf` 참조.

## interfaces/ 원칙

- `@dataclass` 로만 정의된 **순수 데이터 그릇**. 로직 없음.
- 모듈 간 (`acc_can` ↔ `acc_hmi` ↔ `acc_fusion`) 데이터 전달 계약.
- 단위/범위 주석은 **DBC 정합 기준** (범위 변경 시 DBC 먼저, 그다음 dataclass 주석).

## 실행 가이드

### 시뮬레이션 모드 (기본)

```bash
uv run python acc_hmi/main.py
```

- CAN 버스 없음 → `CanInterface._send_raw` 가 `[SIM] TX 0x...` 디버그 로그만 출력
- `acc_state.py` 의 시뮬 상태머신이 GUI 입력에 즉각 반응 (실제 차량 FSM 은 MPC5606B 가 관리)

### 실제 CAN 모드 (RPi + MCP2515/USB-CAN 어댑터 필요)

```python
from acc_can import CanInterface
can = CanInterface(channel="can0", bustype="socketcan")
can.start()  # 실 CAN 버스 연결 시도 → 실패 시 자동 simulation fallback
```

## 요구사항 트레이스

`acc_state.py`, `hmi_gui.py`, `acc_can/*.py` 주석에 SWR/SYS/STK/SAF 박혀 있음. 원문: `../ACC-docs/reqs/STK/SYS/*.sdoc`.

## 편집 시 주의

- **한국어 주석 유지** — UI 라벨·변수 주석·docstring 이 대부분 한국어. 보존.
- **시뮬레이터 ↔ 실제 ECU 분리** — `acc_state.py` 의 상태 전이는 standalone GUI 용. 실제 동작은 MPC5606B 가 결정. 전이 규칙 변경 시 `../ACC-autosar/mbd/AccStateMachine/` 와 `../ACC-docs/reqs/` 도 함께 수정.
- **Python 3.14 prerelease** — `uv` 가 자동 관리. 시스템 파이썬 의존 금지.
- **CAN 코드 수정 시 3 원칙**:
  1. **ID/레이아웃 하드코딩 금지** — `_dbc` / `_codec` 경유
  2. **범위 clamp 는 DBC 기준** — `acc_can/__init__.py` 의 `send_*` 가 DBC 시그널 범위로 max/min
  3. **새 메시지 추가**: DBC 먼저 갱신 → `_dbc.py` 에 `MSG_ID_*` 한 줄 → `_codec.py` 에 encode/decode 함수 → `CanInterface` 에 TX/RX 메서드
- **GUI 변경 테스트** — 타입 체크만으론 부족. `python acc_hmi/main.py` 로 실제 띄워 게이지/페달/버튼/상태 전이 확인.
