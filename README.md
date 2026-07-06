# HYPER

HL FMA 2026 1/5 — ROS 2 기반 자율주행 차량 플랫폼

---

## 패키지 구성

| 패키지 | 설명 |
|--------|------|
| `auto_vehicle` | 차량 제어(Ackermann/조이스틱), 차선 감지 및 시각화 |
| `simulation` | Gazebo 시뮬레이션 월드, 로봇 모델, ROS↔Gazebo 브리지 |

> `simulation` 패키지는 `dev`/`main` 브랜치에서 관리되며, 현재 브랜치(`perception`)의 `src/`에는 `auto_vehicle`만 포함되어 있습니다.

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
colcon build --packages-select simulation
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

## 테스트

이 저장소는 별도의 유닛 테스트 프레임워크 대신 ROS 2 표준인 `ament_lint`(cpplint, uncrustify, lint_cmake 등 정적 분석)를 사용합니다. `package.xml`의 `test_depend`(`ament_lint_auto`, `ament_lint_common`)와 `CMakeLists.txt`의 `BUILD_TESTING` 블록에 설정되어 있습니다.

### 1. 테스트 포함 빌드

`colcon build`는 기본적으로 `BUILD_TESTING`이 켜진 상태로 빌드합니다. 명시적으로 지정하려면:

```bash
colcon build --cmake-args -DBUILD_TESTING=ON
```

### 2. 테스트 실행

```bash
# 전체 패키지 테스트
colcon test

# 특정 패키지만 테스트
colcon test --packages-select auto_vehicle

# 실행 중 결과를 바로 콘솔에 출력
colcon test --packages-select auto_vehicle --event-handlers console_direct+
```

### 3. 결과 확인

```bash
colcon test-result --verbose
```

`--verbose`를 붙이면 실패한 테스트의 상세 로그(어떤 파일의 어떤 라인이 린트 규칙을 위반했는지)까지 함께 출력됩니다.

---

## 시뮬레이션 실행

### 전체 시뮬레이션 한 번에 실행

```bash
ros2 launch simulation full_simulation.launch.py
```

Gazebo 월드, 로봇 스폰, ROS↔Gazebo 브리지, 차선 감지 노드(디버그 대시보드 포함)가 순서대로 실행됩니다.

### 실행 순서 (내부적으로 자동 처리)

| 시각 | 실행되는 것 |
|------|------------|
| t = 0s | Gazebo 월드 (`track.sdf`) 시작 |
| t = 0s | `robot_state_publisher` — URDF 로드 |
| t = 3s | 로봇 엔티티 스폰 (`hyper`) |
| t = 4s | `ros_gz_bridge` — 토픽 브리지 시작 |
| t = 5s | `auto_vehicle`의 `lane_detection` 노드 시작 |

### 실행 옵션

```bash
# 월드 파일 변경
ros2 launch simulation full_simulation.launch.py world:=/path/to/custom.sdf

# 로봇 초기 위치 지정
ros2 launch simulation full_simulation.launch.py x:=2.0 y:=1.0 z:=0.1
```

---

## 차선 감지 노드 개별 실행

시뮬레이션 없이 실제 카메라나 rosbag으로 테스트할 때:

```bash
# 차선 감지 → /lane/center 퍼블리시 + OpenCV 디버그 대시보드 표시
ros2 launch auto_vehicle lane.launch.py
```

---

## 주요 토픽

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/image_raw` | `sensor_msgs/Image` | Gazebo → ROS | 카메라 영상 |
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
