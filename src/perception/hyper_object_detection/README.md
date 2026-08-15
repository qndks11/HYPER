# hyper_object_detection

YOLO 모델로 카메라 영상의 객체와 신호등 상태를 인식하는 Python 패키지입니다. 신호등 결과를 주행 행동 결정에 사용할 수 있도록 문자열 토픽으로 발행합니다.

## 구성

- `object_detection_node`: 영상 입력을 추론하고 `/perception/sign`에 `red`, `green`, `left_arrow`, `none` 등의 결과를 발행합니다.
- `models/best.pt`: 추론에 사용하는 학습 모델입니다.
- `perception.launch.py`: 이 노드와 `hyper_lane_detection`을 함께 실행합니다.

## 실행

```bash
ros2 launch hyper_object_detection perception.launch.py
```
