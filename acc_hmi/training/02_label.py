#!/usr/bin/env python3
"""
Step 2: 간단한 라벨링 도구

촬영한 사진에서 RC카 위치를 마우스로 드래그하여 바운딩 박스를 그립니다.
YOLO 형식 (.txt) 라벨 파일을 자동 생성합니다.

조작법:
  마우스 드래그  - 바운딩 박스 그리기
  SPACE/ENTER   - 현재 박스 저장 & 다음 사진
  R             - 현재 사진 박스 초기화
  S             - RC카 없음으로 스킵 (네거티브 샘플)
  Q             - 종료

완료 후 자동으로 train/val 분할합니다.
"""
import cv2
import os
import glob
import random
import shutil

RAW_DIR = os.path.join(os.path.dirname(__file__), "images", "raw")
TRAIN_IMG = os.path.join(os.path.dirname(__file__), "images", "train")
VAL_IMG = os.path.join(os.path.dirname(__file__), "images", "val")
TRAIN_LBL = os.path.join(os.path.dirname(__file__), "labels", "train")
VAL_LBL = os.path.join(os.path.dirname(__file__), "labels", "val")

for d in [TRAIN_IMG, VAL_IMG, TRAIN_LBL, VAL_LBL]:
    os.makedirs(d, exist_ok=True)

images = sorted(glob.glob(os.path.join(RAW_DIR, "*.jpg")))
if not images:
    print(f"사진이 없습니다! 먼저 01_capture.py를 실행하세요.")
    print(f"찾는 경로: {RAW_DIR}")
    exit(1)

print(f"총 {len(images)}장 라벨링 시작")
print("마우스 드래그=bbox, SPACE=저장&다음, R=리셋, S=스킵, Q=종료")

# Mouse state
drawing = False
ix, iy = 0, 0
boxes = []  # list of (x1, y1, x2, y2) per image
current_box = None


def mouse_cb(event, x, y, flags, param):
    global drawing, ix, iy, current_box
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y
        current_box = None
    elif event == cv2.EVENT_MOUSEMOVE and drawing:
        current_box = (ix, iy, x, y)
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        if abs(x - ix) > 5 and abs(y - iy) > 5:
            current_box = (min(ix, x), min(iy, y), max(ix, x), max(iy, y))
            boxes.append(current_box)
            current_box = None


cv2.namedWindow("Labeling")
cv2.setMouseCallback("Labeling", mouse_cb)

labeled_data = []  # (img_path, boxes_list)
idx = 0

while idx < len(images):
    img_path = images[idx]
    frame = cv2.imread(img_path)
    h, w = frame.shape[:2]
    boxes = []
    current_box = None
    fname = os.path.basename(img_path)

    while True:
        display = frame.copy()
        # Draw saved boxes
        for (x1, y1, x2, y2) in boxes:
            cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
        # Draw current dragging box
        if current_box:
            cv2.rectangle(display, (current_box[0], current_box[1]),
                          (current_box[2], current_box[3]), (0, 200, 255), 2)

        cv2.putText(display, f"[{idx+1}/{len(images)}] {fname}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(display, f"Boxes: {len(boxes)}", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        cv2.putText(display, "SPACE=save  R=reset  S=skip  Q=quit", (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
        cv2.imshow("Labeling", display)

        key = cv2.waitKey(30) & 0xFF
        if key == ord(" ") or key == 13:  # SPACE or ENTER
            labeled_data.append((img_path, list(boxes)))
            print(f"  [{idx+1}] {fname} → {len(boxes)} box(es)")
            idx += 1
            break
        elif key == ord("r"):
            boxes = []
            current_box = None
        elif key == ord("s"):
            # Skip = negative sample (no boxes)
            labeled_data.append((img_path, []))
            print(f"  [{idx+1}] {fname} → skipped (negative)")
            idx += 1
            break
        elif key == ord("q"):
            idx = len(images)  # exit outer loop
            break

cv2.destroyAllWindows()

if not labeled_data:
    print("라벨링된 데이터가 없습니다.")
    exit(1)

# Split train/val (80/20)
random.shuffle(labeled_data)
split = int(len(labeled_data) * 0.8)
train_data = labeled_data[:split]
val_data = labeled_data[split:]

def save_split(data, img_dir, lbl_dir):
    for img_path, bboxes in data:
        fname = os.path.basename(img_path)
        base = os.path.splitext(fname)[0]

        # Copy image
        shutil.copy2(img_path, os.path.join(img_dir, fname))

        # Write YOLO label (class x_center y_center width height, normalized)
        img = cv2.imread(img_path)
        h, w = img.shape[:2]
        label_path = os.path.join(lbl_dir, base + ".txt")
        with open(label_path, "w") as f:
            for (x1, y1, x2, y2) in bboxes:
                cx = ((x1 + x2) / 2) / w
                cy = ((y1 + y2) / 2) / h
                bw = (x2 - x1) / w
                bh = (y2 - y1) / h
                f.write(f"0 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n")

save_split(train_data, TRAIN_IMG, TRAIN_LBL)
save_split(val_data, VAL_IMG, VAL_LBL)

print(f"\n라벨링 완료!")
print(f"  Train: {len(train_data)}장")
print(f"  Val:   {len(val_data)}장")
print(f"\n다음 단계: python 03_train.py")
