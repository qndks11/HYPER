# hyper_control

HYPER 차량의 기본 모델과 저수준 제어 노드를 제공하는 패키지입니다. URDF/Xacro 차량 모델, 조향·속도 제어 노드, 조이스틱 텔레옵 launch를 포함합니다.

## 구성

- `vehicle_controller_node`: `/steering_angle`, `/velocity` 명령을 받아 차량 조향과 구동을 제어합니다.
- `joystick_controller_node`: 조이스틱 입력을 위 두 제어 토픽으로 변환합니다(축만 읽습니다).
- `estop_controller_node`: 조이스틱 버튼으로 차를 세우고 미션을 일시정지합니다.
- `urdf/vehicle.xacro`: 차량 링크·조인트 모델입니다.

## `estop_controller_node` -- 조이스틱 정지/재개

버튼 두 개입니다. 눌린 순간(rising edge)만 봅니다.

| 파라미터 | 기본값 | 하는 일 |
| --- | --- | --- |
| `estop_button_index` | 1 | 세우고 미션을 일시정지 |
| `resume_button_index` | 0 | 풀고 미션을 이어 감 |

**기본값 1/0은 확인된 값이 아닙니다.** 패드마다 다르므로 `ros2 topic echo /joy`로 실제
인덱스를 보고 launch 인자로 덮어쓰세요.

정지 버튼을 누르면 두 가지가 같이 일어납니다.

1. `/estop`(`std_msgs/Bool`, latched)을 `true`로 발행합니다. 실차에서 차를 실제로 세우는
   것은 이쪽입니다 -- `hyper_interface`의 `arduino_interface_node`가 `/estop`이 `true`인
   동안 `/velocity`와 `/steering_angle`을 무시하고 0을 내보냅니다. **그래서 일시정지 중에는
   조이스틱 수동 주행도 막힙니다.** 차를 손으로 옮기려면 재개 버튼을 먼저 누르세요.
2. `/mission_manager/pause`를 부릅니다(`~/cancel`이 아닙니다). 미션은 취소되지 않고 그
   스텝에 남은 시간까지 그대로 멈춰 있다가, 재개 버튼이 `/mission_manager/resume`을 부르면
   멈춘 자리에서 이어 갑니다. 자세한 것은 `hyper_planner/README.md`의 `~/pause` / `~/resume`
   절을 보세요.

`mission_manager`가 안 떠 있어도(수동 주행 launch 트리) 문제 없습니다 -- 서비스 호출은
best-effort이고, 정지 자체는 `/estop`으로 downstream에서 일어납니다.

이 노드는 `/velocity`나 `/steering_angle`을 발행하지 않습니다. 그래서 미션 주행 중
`cmd_vel_to_ackermann_node`와 같이 띄워도 토픽이 겹치지 않습니다(`joystick_controller_node`는
겹칩니다).

## 실행

```bash
ros2 launch hyper_control joystick.launch.py
ros2 launch hyper_control estop.launch.py launch_joy_node:=true   # 정지/재개 버튼만
```

Gazebo 시뮬레이션은 이 패키지의 모델과 컨트롤러를 사용하며, 실행은 `hyper_gazebo` 또는 `hyper_launch`에서 담당합니다.
