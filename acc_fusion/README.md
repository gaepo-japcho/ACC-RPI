# acc_fusion

## Demo 실행

반드시 **프로젝트 루트**에서 실행해야 합니다.

```bash
cd /path/to/ACC-RPI

python -m acc_fusion.demo.cam      # 카메라
python -m acc_fusion.demo.lidar    # 라이다
python -m acc_fusion.demo.yolo_cam # 카메라 + YOLO
python -m acc_fusion.demo.fusion   # 전체 퓨전
```

> `acc_fusion/demo/` 안에서 `python cam.py` 로 직접 실행하면 상대 import 오류가 발생합니다.
