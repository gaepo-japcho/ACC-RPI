# CLAUDE.md — ACC-RPI

이 저장소는 ACC 스케일카의 **Raspberry Pi 5** 측 코드다. HMI(PyQt5 GUI), 카메라 + LiDAR 센서 퓨전, CAN 브릿지를 담당한다. 독립 git repo. 상위 맥락은 `/mnt/c/acc/CLAUDE.md` 참조.

## 디렉토리 구조

```
ACC-RPI/
├── main.py                     # ★ 진짜 진입점 아님 — 모듈 import 만 하는 스텁
├── pyproject.toml              # uv 기반 의존성 정의 (>= Python 3.14)
├── uv.lock
├── .python-version             # "3.14" (실제 릴리스 버전)
├── README.md                   # 현재 비어 있음
├── acc_hmi/                    # ★ 실제 진입점이 있는 곳
│   ├── main.py                 # hmi_gui.gui_main() 호출
│   ├── hmi_gui.py              # PyQt5 GUI (~780 라인, 커스텀 게이지 · 페달)
│   ├── acc_state.py            # 상태머신 시뮬레이션 + AccData 데이터 모델
│   ├── can_interface.py        # SimulatedCanInterface / RealCanInterface
│   ├── requirements.txt        # PyQt5, opencv, ultralytics 등 (acc_hmi 전용)
│   └── training/               # YOLO 데이터셋 캡처 · 라벨링 · 학습 스크립트
├── acc_can/                    # ★ 현재 __init__.py 뿐인 스켈레톤
│   └── __init__.py
└── acc_fusion/                 # ★ 현재 __init__.py 뿐인 스켈레톤
    └── __init__.py
```

## 진입점

- 루트 `main.py` 는 스텁 — 실행해도 아무것도 안 일어남. 실제 GUI 를 띄우려면 `acc_hmi/main.py` 를 써야 한다.

```bash
cd ACC-RPI
uv sync                         # 루트 pyproject.toml 기반 설치

cd acc_hmi
pip install -r requirements.txt # PyQt5 등 HMI 전용 deps (pyproject.toml 과 분리됨)
python main.py                  # 시뮬레이션 모드로 GUI 실행
```

**`--can` 플래그에 대한 주의 사항** — 루트 CLAUDE.md 및 `acc_hmi/main.py` docstring 에는 `python main.py --can` 이 "실제 CAN 모드" 로 기술되어 있으나, **현재 코드에는 argparse 가 없다**. `main.py` 는 플래그를 무시하고 항상 `gui_main()` → `SimulatedCanInterface` 로 동작한다. 실제 CAN 연결을 원하면 `--can` 파싱 + `RealCanInterface` 스위칭 로직을 추가해야 한다.

## 의존성 분리 구조 (주의)

의존성 정의가 **두 곳으로 분리**되어 있다:

| 파일 | 관리 도구 | 내용 |
|---|---|---|
| `/pyproject.toml` + `uv.lock` | `uv` | `numpy`, `opencv-python`, `ultralytics` |
| `/acc_hmi/requirements.txt` | `pip` | `PyQt5>=5.15`, `opencv`, `ultralytics`, `numpy`; `python-can>=4.0` 은 주석 처리됨 |

- **PyQt5 는 pyproject.toml 에 없다** — `acc_hmi/` 전용. HMI 작업 시 `requirements.txt` 도 반드시 설치.
- `python-can` 은 실제 CAN 모드에서만 필요하며 현재 주석 처리. 활성화 시 MCP2515 또는 USB-CAN 어댑터가 있어야 함.
- Python **3.14** 이상 (`.python-version`, `pyproject.toml`). 3.13 이하에서는 uv 가 거부한다.

## acc_hmi 구성

- **`acc_state.py`** — `AccState` enum (`OFF / STANDBY / CRUISING / FOLLOWING / FAULT`), `AccData` dataclass, `AccController` 상태 전이. **이 상태머신은 시뮬레이션용**이다. 실제 차량에서는 MPC5606B 가 상태를 관리하고 RPi 는 렌더링만 한다. 요구사항이 바뀌면 MPC5606B 쪽 Simulink 모델 (`../ACC-autosar/mbd/`) 과 동기화할 것.
- **`hmi_gui.py`** — 커스텀 arc 게이지(속도, PWM), 세로 슬라이더 페달, GAP 아이콘(3-level), 버튼 그리드(ACC / CANCEL / SPEED+/- / SET/RES / GAP / BRAKE / ACCEL), SIM 패널(차량 토글, ±SPD, FAULT 트리거). 50 ms 타이머로 리프레시. 리소스 파일(`.ui`, `.qrc`)은 없고 모두 코드 기반.
- **`can_interface.py`** — 두 구현:
  - `SimulatedCanInterface`: 루프백 스텁 (현재 기본).
  - `RealCanInterface`: `python-can` 사용. 현재 **placeholder ID `0x200` / `0x201`** 을 쓰고 있으며 코드 주석에 "match actual DBC later" 로 명시되어 있다. 실제 DBC 확정 시 수정 필요.

## CAN ID 설계 의도 vs 코드 현황

루트 CLAUDE.md / 아키텍처 문서가 정의한 ID 와 현재 `can_interface.py` 가 쓰는 ID 가 다르다. **아키텍처가 진실의 원천**이며 코드를 맞춰가야 한다:

| 메시지 | 설계 (DBC / ACC-docs) | 코드 현황 | 주기 |
|---|---|---|---|
| SENSOR_FUSION (RPi→ECU, 카메라/LiDAR 결과) | `0x110` | 미구현 | 20 ms |
| SENSOR_HEARTBEAT (RPi→ECU, HB+ERR) | `0x111` | 미구현 | 20 ms |
| GUI_CTRL (RPi→ECU, 버튼 입력) | `0x510` | `0x200` placeholder | 50 ms |
| GUI_STATUS (ECU→RPi, 상태) | `0x520` | `0x201` placeholder | 50 ms |
| ECU_HEARTBEAT (ECU→RPi, HB+ERR) | `0x410` | 미구현 | 10 ms |

DBC 확정 시 `can_interface.py` 의 상수 + `encode_driver_input()` 호출부를 함께 수정. `encode_driver_input()` 은 정의만 되어 있고 아직 호출되지 않는다.

## 스켈레톤 모듈 (미구현)

- **`acc_can/`** — CAN 브릿지 모듈. 현재 `__init__.py` 만 있음. 장기적으로 `can_interface.py` 의 `RealCanInterface` 와 결합해 이 모듈로 이관되는 것이 자연스럽다.
- **`acc_fusion/`** — 카메라 + LiDAR 퓨전. 현재 `__init__.py` 만 있음. MRR-30 LiDAR 스펙은 `../ACC-docs/refs/20220901 MRR-30 User Manual & Cert List_00.pdf` 참조. YOLO 는 `acc_hmi/training/` 에 학습 파이프라인이 먼저 들어와 있다.

## 요구사항 트레이스

`acc_state.py`, `hmi_gui.py`, `can_interface.py` 주석에 요구사항 ID(SWR / SYS / STK) 가 풍부하게 박혀 있다. 예:
- `acc_state.py`: SWR001–SWR003, SWR016, SWR020–SWR031; SYS001–SYS031; STK009, STK020–STK024
- `hmi_gui.py`: STK020, STK021, SYS018, SYS019, SYS029, SWR018, SWR019
- `can_interface.py`: SYS030, SYS031, SWR020, SWR030

원문은 `../ACC-docs/reqs/STK/SYS/*.sdoc`. 수정 · 삭제 · 추가 시 .sdoc 쪽 번호와 동기화 유지.

## 편집 시 주의

- **한국어 주석 유지** — UI 라벨("차량", "브레이크", "가로 배치 GAP 아이콘" 등), 변수 주석, 메서드 docstring 이 대부분 한국어. 보존.
- **시뮬레이터와 실제 ECU 분리 의식** — `acc_state.py` 의 상태머신은 standalone GUI 테스트용. 실제 동작은 MPC5606B 가 결정. 상태 전이 규칙을 바꾸려면 AUTOSAR 쪽(`../ACC-autosar/mbd/AccStateMachine.slx`) 과 `../ACC-docs/autosar/ACC_AUTOSAR_Architecture_Sketch.md` 를 함께 수정해야 일관성이 유지된다.
- **Python 3.14 prerelease** — `uv` 가 자동 관리. 시스템 파이썬에 의존하지 말 것.
- **GUI 변경 테스트** — 타입체크/린트만으로는 부족. 실제로 `python main.py` 로 띄워서 게이지 갱신 · 페달 입력 · 상태 전이가 의도대로 움직이는지 확인.
