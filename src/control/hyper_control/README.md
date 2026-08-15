# hyper_control

HYPER 차량의 기본 모델과 저수준 제어 노드를 제공하는 패키지입니다. URDF/Xacro 차량 모델, 조향·속도 제어 노드, 조이스틱 텔레옵 launch를 포함합니다.

## 구성

- `vehicle_controller_node`: `/steering_angle`, `/velocity` 명령을 받아 차량 조향과 구동을 제어합니다.
- `joystick_controller_node`: 조이스틱 입력을 위 두 제어 토픽으로 변환합니다.
- `urdf/vehicle.xacro`: 차량 링크·조인트 모델입니다.

## 실행

```bash
ros2 launch hyper_control joystick.launch.py
```

Gazebo 시뮬레이션은 이 패키지의 모델과 컨트롤러를 사용하며, 실행은 `hyper_gazebo` 또는 `hyper_launch`에서 담당합니다.
