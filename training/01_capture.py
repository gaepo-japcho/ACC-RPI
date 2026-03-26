#!/usr/bin/env python3
"""
Step 1: RC카 뒷모습 사진 수집 도구

카메라를 켜고 스페이스바를 누르면 사진을 저장합니다.
다양한 거리/각도/조명에서 50~100장 이상 촬영하세요.

조작법:
  SPACE  - 사진 촬영
  Q      - 종료
"""
import cv2
import os
import time

SAVE_DIR = os.path.join(os.path.dirname(__file__), "images", "raw")
os.makedirs(SAVE_DIR, exist_ok=True)

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("카메라를 열 수 없습니다!")
    exit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

count = len([f for f in os.listdir(SAVE_DIR) if f.endswith(".jpg")])
print(f"저장 폴더: {SAVE_DIR}")
print(f"기존 사진: {count}장")
print("SPACE=촬영, Q=종료")
print()
print("팁: 다양한 조건에서 촬영하세요")
print("  - 가까이/멀리 (20cm ~ 2m)")
print("  - 정면/약간 비스듬히")
print("  - 밝은 곳/어두운 곳")
print("  - RC카가 화면 중앙/좌측/우측")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    display = frame.copy()
    cv2.putText(display, f"Captured: {count}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(display, "SPACE=capture  Q=quit", (10, 460),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
    cv2.imshow("RC Car Capture", display)

    key = cv2.waitKey(1) & 0xFF
    if key == ord(" "):
        filename = f"rc_{int(time.time())}_{count:04d}.jpg"
        filepath = os.path.join(SAVE_DIR, filename)
        cv2.imwrite(filepath, frame)
        count += 1
        print(f"  [{count}] {filename}")
    elif key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
print(f"\n총 {count}장 촬영 완료!")
print(f"다음 단계: python 02_label.py")
