# hyper_interface

ROS 2와 Arduino 보드 사이의 시리얼 인터페이스 패키지입니다. `hyper_control`(시뮬레이션)과
`hyper_planner`(실차)가 이미 사용하는 `/velocity`[m/s], `/steering_angle`[rad] 명령 토픽을
그대로 구독해서 Arduino로 전달하고, Arduino는 그 값을 받아 **L298N 듀얼 H-브릿지 모터드라이버**
(채널 A = 구동 모터, 채널 B = 조향 모터)를 직접 구동합니다.

**아직 구동 모터에 엔코더, 조향에 위치 센서가 없는 상태**이므로 폐루프 제어가 아니라
명령값을 PWM duty로 선형 매핑만 하는 오픈루프 구조입니다. 나중에 엔코더/조향각 센서를
달면 `arduino/hyper_motor_interface/hyper_motor_interface.ino`에 PID 등 피드백 제어를
추가하면 됩니다.

## 구성

- `hyper_interface/arduino_interface_node.py`: `/velocity`, `/steering_angle`을 구독해서
  일정 주기(`send_rate`, 기본 50 Hz)로 Arduino에 시리얼 패킷을 전송하는 ROS 2 노드입니다.
  토픽이 `command_timeout`(기본 0.3 s) 이상 끊기면 0을 보내는 안전장치가 있습니다.
- `arduino/hyper_motor_interface/hyper_motor_interface.ino`: Arduino 보드에 올리는 펌웨어.
  패킷을 파싱해서 L298N의 ENA/IN1/IN2(구동), ENB/IN3/IN4(조향) 핀을 제어합니다. 마찬가지로
  패킷이 `COMMAND_TIMEOUT_MS`(기본 300 ms) 이상 끊기면 모터를 정지시킵니다.
- `config/parameters.yaml`, `launch/interface.launch.py`: 노드 실행용 설정/launch 파일.

## 통신 프로토콜

ROS 2 → Arduino, 11바이트 리틀엔디안 고정 길이 패킷을 `send_rate` 주기로 계속 전송합니다
(패킷 자체가 워치독 heartbeat 역할을 겸함):

| byte  | 내용                                  |
|-------|---------------------------------------|
| 0     | `0xAA` (start-of-frame 1)             |
| 1     | `0x55` (start-of-frame 2)             |
| 2-5   | `float32` velocity `[m/s]`            |
| 6-9   | `float32` steering_angle `[rad]`      |
| 10    | checksum = byte 2~9 XOR                |

`arduino_interface_node.py`의 `_make_packet()`과 `.ino`의 파싱 로직이 이 형식을 공유하므로,
한쪽을 바꾸면 반드시 다른 쪽도 같이 맞춰야 합니다.

## 배선

기본 핀 배정(`hyper_motor_interface.ino` 상단 `const uint8_t ..._PIN` 값에서 변경 가능)은
L298N 한 장으로 구동/조향 두 모터를 모두 구동하는 구성 기준입니다. 채널 A(OUT1/OUT2)에
구동 모터, 채널 B(OUT3/OUT4)에 조향 모터를 연결하세요. 모터 전원(+12V/GND)은 L298N의
전원 입력단에, 로직 5V/GND는 Arduino와 공통 GND로 연결합니다(모터 전원이 12V 이하면
L298N 보드의 5V 레귤레이터 점퍼를 꽂아 별도 5V 없이 로직 전원을 공급받을 수도 있습니다).

| Arduino 핀 | L298N 핀 | 역할              |
|-----------|----------|-------------------|
| D9        | ENA      | 구동 모터 PWM      |
| D8        | IN1      | 구동 모터 방향 1   |
| D7        | IN2      | 구동 모터 방향 2   |
| D10       | ENB      | 조향 모터 PWM      |
| D12       | IN3      | 조향 모터 방향 1   |
| D11       | IN4      | 조향 모터 방향 2   |

> L298N은 트랜지스터 전압 강하(~2V)가 있고 연속 전류도 2A 안팎으로 넉넉하지 않습니다.
> 프로토타입 단계에서는 괜찮지만, 구동 모터가 부하를 받아 전류가 더 필요해지면 BTS7960 같은
> MOSFET 드라이버로 바꾸는 것을 고려하세요 (그 경우 `.ino`의 `DRIVE_EN/IN1/IN2` 쪽만
> 수정하면 됩니다).

## 실행

1. Arduino IDE로 `arduino/hyper_motor_interface/hyper_motor_interface.ino`를 보드에 업로드합니다.
2. `python3-serial` 설치 확인: `sudo apt install python3-serial` (또는 `pip install pyserial`).
3. 연결된 포트 확인: `ls -l /dev/tty_arduino`. 이 이름은 저장소의
   `udev/99-hyper-serial.rules`가 만드는 고정 심볼릭 링크입니다(루트 README의
   "USB 시리얼 포트 고정" 절). 이 보드는 CH340 칩셋이라 `/dev/ttyUSB*`로 잡히는데
   IMU 어댑터도 같은 자리를 노리므로 번호로 잡으면 안 됩니다
   (네이티브 USB나 FTDI 보드는 `/dev/ttyACM*`) -- `lsusb`로 "QinHeng Electronics CH340"이
   보이는지 확인.
4. 노드 실행:

```bash
ros2 launch hyper_interface interface.launch.py     # 기본 포트 /dev/tty_arduino
```

`/velocity`, `/steering_angle`에 직접 퍼블리시하거나(`ros2 topic pub ...`), 조이스틱
(`hyper_control`의 `joystick_controller_node`) 또는 `hyper_planner`의 컨트롤러를 함께
띄우면 그 명령이 그대로 Arduino로 전달됩니다.

## 참고

- `real.launch.py`(`hyper_launch`)는 아직 이 노드를 포함하지 않습니다. 실차에서 상시
  구동하려면 `real.launch.py`의 `behavior.launch.py` 단계 뒤에 `interface.launch.py`를
  추가하세요.
- `max_velocity`, `max_steering_angle`은 `hyper_control`/`hyper_planner`의 동일 파라미터와
  값을 맞춰야 Arduino가 명령 범위 밖의 값을 받는 일이 없습니다.
