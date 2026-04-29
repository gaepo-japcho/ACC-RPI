# acc_track_fast_fusion

`acc_fast_fusion` 위에 OpenCV 기반 트래커 한 층을 더한 fusion 모듈. 공개 API 동일 (`from acc_track_fast_fusion import Fusion`).

## 왜?

`acc_fast_fusion` 은 NPU 갱신 사이의 카메라 frame 들에서 bbox 가 멈춰 보임 (가장 최근 NPU 결과를 그대로 표시). 차량이 빠르게 움직이거나 NPU 가 한두 frame 놓치면 bbox 가 늦거나 깜빡임. 이 모듈은:

- **매 카메라 frame 마다 트래커 update** — bbox 가 차량 움직임을 따라감
- **NPU 갱신 시 IoU 매칭 reconcile** — YOLO 가 진실, 트래커는 보간

## 사라짐 정책 (사용자 명시)

> 카메라에서 차량이 없어지면 바로 없어져야 한다. 다시 나타날 걸 예측해서 이상한 데에 bbox 두면 안 된다.
> 잠깐 YOLO 가 못 잡아도 tracker 가 메꿔줘야 한다.

두 요구가 살짝 텐션이 있어서 layer 를 분리:

1. **트래커 layer — bbox 가 화각 밖으로 벗어날 때만 invalidate**
   - bbox 의 가시 영역이 10% 미만이면 fail.
   - search window 가 화면 끝에 부딪혀 template 보다 작아지면 fail.
   - 매칭 score 가 낮은 frame 에서는 위치 갱신만 skip — 트랙은 살리고 직전 bbox 유지.
     (잠깐 가려짐/모션 블러에 강건. score 가 회복되면 다시 따라감.)
   - 위치 예측 없음 — search window 는 직전 bbox 주변(±50%) 한정. bbox 가 임의의 곳으로
     점프하지 않음.
2. **YOLO reconcile layer — 매칭 안 된 트랙 즉시 제거**
   - YOLO 가 결과 내놓을 때마다 IoU 매칭 (threshold=0.2) 으로 진실 보정.
   - 매칭 안 된 트랙(= YOLO 가 해당 차량 더 이상 안 봄)은 즉시 폐기.
   - 이게 "차량이 화면에서 사라지면 즉시 bbox 제거" 의 진짜 보장.

**결과**:
- YOLO 사이의 frame 들에서는 트래커가 bbox 를 부드럽게 따라감.
- 차량이 화면 안에서 잠깐 가려져도 트랙 보존 (직전 위치 유지).
- 차량이 진짜 사라지면 다음 YOLO 사이클(~30ms) 에 bbox 제거.
- 차량이 화각 밖으로 빠지면 트래커 layer 에서 즉시 fail.

## bbox → LiDAR 매핑

`bbox_to_distance_mm(bbox, scan_points, hfov_deg, img_w)`:

```
카메라 좌표:    x=0 좌측 ──── x=W/2 광축 ──── x=W 우측
                  ↓             ↓              ↓
LiDAR 각도(0..360): 360-hfov/2    0°           +hfov/2
```

bbox 의 (x1, x2) 만 사용 (bbox 의 y 는 거리 추정에 무관 — LiDAR 가 1D 평면 스캔). 해당 각도 범위의 LiDAR 점들 중 `min(distance_mm)` 채택. 여러 트랙 있으면 가장 가까운 차량 거리가 `FusionData.distance` 로 push.

`hfov`/`img_w` 는 `config.toml [camera]` 의 진실 그대로 사용.

## 트래커 구현 (ACC 특화)

`tracker._BboxTracker` — `cv2.TrackerCSRT` wrapper (opencv-contrib-python 필요).

ACC 시나리오 핵심: **차량 거리 변화 → bbox 스케일 변화**. CSRT 는 이 특성을 정면에서 다룸:

1. **DSST-style scale estimator 내장** — 매 update 에서 별도 1D filter 로 스케일 추정.
   자체 multi-scale brute force 보다 안정적이고 노이즈에 강건.
2. **HOG + Color Names + grayscale 멀티채널** — 단일 grayscale matchTemplate 보다
   외관 변화/조명 변화에 robust.
3. **Spatial reliability map** — bbox 안에서 신뢰할 픽셀 영역만 가중치. 배경 흡수
   drift 줄어듦.
4. **외관 갱신** — CSRT 가 내부에서 처리. 우리는 추가로 YOLO reseed (~30ms 주기)
   로 진실 보정.

이전 자체 multi-scale matchTemplate 구현은 합성 검증에서 차량이 멀어질 때 bbox 가
오히려 커지는 drift 발견 — CSRT 로 교체.

## 실행

데모 (트래커 시각 확인):
```bash
uv run python -m acc_track_fast_fusion.demo.yolo_cam
```

전체 시스템 (HMI + CAN + LiDAR fusion):
```python
# main.py 의 한 줄만 교체
# from acc_fusion import Fusion           # 동기, 트래커 없음
# from acc_fast_fusion import Fusion      # 비동기 NPU, 트래커 없음
from acc_track_fast_fusion import Fusion  # ← 비동기 NPU + 트래커 + LiDAR 매핑
```

## HUD (데모)

좌상단 4 줄:
- `capture` — 카메라 입력 처리 fps
- `yolo` — NPU 결과 갱신 fps (트래커 reseed 빈도)
- `yolo lag` — 현재 frame 과 마지막 YOLO 결과 frame 의 차이
- `tracks` — 현재 활성 트랙 수

bbox 라벨: `#{track_id} {class_name} {confidence}`. 사라졌다 다시 잡히는 차량은 새 ID 가 발급됨 (예측 grace 없는 정책의 자연스러운 귀결).

## 주의

- `acc_fusion` / `acc_fast_fusion` / `acc_track_fast_fusion` 중 **하나만** 사용. 동시에 두 개 이상 import 하면 NPU VDevice group_id 가 달라 충돌 가능.
- 의존성: `opencv-contrib-python` (CSRT 필요). `opencv-python` 와 충돌하므로 둘 중 하나만.
- `tracker.py` 상단 튜닝 상수:
  - `_VISIBLE_AREA_RATIO=0.1` — bbox 가시 영역이 이 미만이면 트래커 fail (화각 밖).
  - `_IOU_MATCH_THRESHOLD=0.2` — YOLO reconcile 시 트랙↔검출 매칭 IoU 하한.
  - CSRT 자체 파라미터는 `cv2.TrackerCSRT_Params()` 로 조정 가능 (현재는 OpenCV 기본값).
- LiDAR 매핑은 카메라 광축과 LiDAR 0° 가 정렬돼 있다고 가정. 어긋나면 `config.toml [lidar] angle_offset` 으로 보정 (이미 LidarReader 가 적용 중).
