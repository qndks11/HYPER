# HYPER

HL FMA 2026 1/5 — ROS 2 기반 자율주행 차량 플랫폼

---

## 패키지 구성

| 패키지 | 설명 |
|--------|------|
| `auto_vehicle` | 차량 제어(Ackermann/조이스틱), Gazebo 시뮬레이션, 차선 감지 및 시각화 |

### `auto_vehicle` 구성 요소

| 실행 파일 (노드) | 소스 | 설명 |
|------------------|------|------|
| `vehicle_controller` | `src/vehicle_controller.cpp` | Ackermann 조향/속도 명령 처리 |
| `joystick_controller` | `src/joystick_controller.cpp` | 조이스틱 입력 → `/cmd_vel` 변환 |
| `lane_detection` | `src/lane_detection.cpp` | 카메라 영상 기반 차선 감지 + OpenCV 디버그 대시보드 |

---

## 프로젝트 파일 구조

```
HYPER/
├── src/
│   └── auto_vehicle/
│       ├── config/                 # 파라미터 및 ROS↔Gazebo 브리지 설정 (YAML)
│       │   ├── gz_ros2_control.yaml
│       │   ├── parameters.yaml
│       │   └── ros_gz_bridge.yaml
│       ├── include/auto_vehicle/   # 헤더 파일
│       ├── launch/                 # ROS 2 launch 파일
│       │   ├── joystick.launch.py
│       │   ├── lane.launch.py
│       │   └── vehicle.launch.py
│       ├── src/                    # 노드 소스 코드 (위 표 참고)
│       ├── urdf/                   # 로봇 모델 (URDF/xacro)
│       ├── worlds/                 # Gazebo 월드(SDF) 및 모델
│       ├── CMakeLists.txt
│       └── package.xml
├── docs/
│   └── pull_request_guide.md       # PR 작성 가이드
└── README.md
```

---

## 빌드 방법 (colcon)

### 1. 의존성 설치

```bash
cd ~/HYPER
rosdep install --from-paths src --ignore-src -r -y
```

### 2. 빌드

```bash
colcon build
```

특정 패키지만 빌드하려면:

```bash
colcon build --packages-select auto_vehicle
```

### 3. 환경 소싱 (source)

빌드 후 **터미널을 새로 열 때마다** 아래 명령을 실행해야 합니다:

```bash
source ~/HYPER/install/setup.bash
```

매번 입력하기 번거로우면 `~/.bashrc`에 추가:

```bash
echo "source ~/HYPER/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 시뮬레이션 실행

### Gazebo 시뮬레이션 + 차량 제어 실행

```bash
ros2 launch auto_vehicle vehicle.launch.py
```

Gazebo 월드, 로봇 스폰, `robot_state_publisher`, `vehicle_controller`, `ros_gz_bridge`가 실행되고, 로봇 스폰이 끝나면 `ros2_control` 컨트롤러들이 순서대로 활성화됩니다.

### 실행 순서 (이벤트 기반으로 자동 처리)

| 순서 | 실행되는 것 |
|------|------------|
| 1 | Gazebo 월드 (`track.world`) 시작 + `robot_state_publisher`, `vehicle_controller`, `ros_gz_bridge` 동시 실행 |
| 2 | 로봇 엔티티 스폰 (`ackermann_steering_vehicle`) |
| 3 | 스폰 완료 → `joint_state_broadcaster` 활성화 |
| 4 | 활성화 완료 → `forward_velocity_controller`, `forward_position_controller` 활성화 |

### 실행 옵션

```bash
# 월드 파일 변경
ros2 launch auto_vehicle vehicle.launch.py world:=/path/to/custom.sdf

# 로봇 초기 위치/자세 지정 (x, y, z, roll(R), pitch(P), yaw(Y))
ros2 launch auto_vehicle vehicle.launch.py x:=2.0 y:=1.0 z:=0.1 R:=0.0 P:=0.0 Y:=0.0
```

---

## 차선 감지 노드 실행

`vehicle.launch.py`는 차선 감지 노드를 포함하지 않으므로, 별도로 실행해야 합니다. 시뮬레이션 카메라(`/camera/image_raw`)와 실제 카메라/rosbag 모두 동일하게 사용할 수 있습니다.

```bash
# 차선 감지 → /lane/center 퍼블리시 + OpenCV 디버그 대시보드 표시
ros2 launch auto_vehicle lane.launch.py
```

시뮬레이션과 함께 사용하려면 `vehicle.launch.py`를 먼저 실행한 뒤, 별도 터미널에서 위 명령을 실행하면 됩니다.

---

## 주요 토픽

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo → ROS | 카메라 영상 |
| `/lane/center` | `std_msgs/Float64MultiArray` | auto_vehicle → | `[offset_m, steering_angle_deg, curvature_px, valid]` |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → Gazebo | 속도 명령 |
| `/odom` | `nav_msgs/Odometry` | Gazebo → ROS | 오도메트리 |
| `/imu` | `sensor_msgs/Imu` | Gazebo → ROS | IMU |

---

## 브랜치 구조

```
main          ← 안정 버전. 직접 push 금지, PR로만 머지.
  ├── perception   ← 인지 스택 (현재 브랜치)
  ├── control      ← 차량 제어
  ├── planning     ← 경로 계획
  └── dev          ← 통합 테스트
```
