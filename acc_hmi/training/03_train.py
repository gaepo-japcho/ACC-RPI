#!/usr/bin/env python3
"""
Step 3: YOLO 파인튜닝

COCO pretrained 모델을 기반으로 RC카 뒷모습 감지를 학습합니다.
50~100장이면 충분히 좋은 성능이 나옵니다.

완료 후 best.pt를 HMI에서 사용합니다.
"""
import os
from ultralytics import YOLO

BASE_DIR = os.path.dirname(__file__)
DATASET_YAML = os.path.join(BASE_DIR, "dataset.yaml")
# HMI 폴더의 base model 사용
BASE_MODEL = os.path.join(BASE_DIR, "..", "yolo26n.pt")

if not os.path.exists(BASE_MODEL):
    # fallback
    BASE_MODEL = "yolo11n.pt"
    print(f"yolo26n.pt 없음, {BASE_MODEL} 사용")

# Check dataset
train_imgs = os.path.join(BASE_DIR, "images", "train")
n_train = len([f for f in os.listdir(train_imgs) if f.endswith(".jpg")]) if os.path.exists(train_imgs) else 0
if n_train == 0:
    print("학습 이미지가 없습니다!")
    print("먼저 01_capture.py → 02_label.py를 실행하세요.")
    exit(1)

print(f"학습 이미지: {n_train}장")
print(f"베이스 모델: {BASE_MODEL}")
print(f"데이터셋: {DATASET_YAML}")
print()

# Load pretrained model
model = YOLO(BASE_MODEL)

# Fine-tune
results = model.train(
    data=DATASET_YAML,
    epochs=50,            # 50~100장이면 50 epoch이면 충분
    imgsz=640,
    batch=8,              # RPi/Jetson이면 batch=4로 줄이기
    patience=10,          # early stopping
    project=os.path.join(BASE_DIR, "runs"),
    name="rc_car_detect",
    exist_ok=True,
    # Transfer learning: freeze backbone early layers
    freeze=10,
    # Augmentation (소량 데이터에 중요)
    hsv_h=0.015,
    hsv_s=0.7,
    hsv_v=0.4,
    flipud=0.0,           # 상하반전은 의미 없음
    fliplr=0.5,           # 좌우반전은 유용
    mosaic=1.0,
    mixup=0.1,
)

# Copy best model to HMI directory
best_pt = os.path.join(BASE_DIR, "runs", "rc_car_detect", "weights", "best.pt")
dest_pt = os.path.join(BASE_DIR, "..", "rc_car_best.pt")

if os.path.exists(best_pt):
    import shutil
    shutil.copy2(best_pt, dest_pt)
    print(f"\n학습 완료!")
    print(f"모델 저장: {dest_pt}")
    print("\n현재 앱에서는 카메라 추론을 사용하지 않습니다.")
    print("필요하면 이후 카메라 입력 기능을 다시 연결해 모델 경로를 반영하세요.")
else:
    print("학습 실패 - best.pt를 찾을 수 없습니다")
