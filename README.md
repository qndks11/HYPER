# HYPER

HL FMA 2026 1/5 — ROS 2 기반 자율주행 차량 플랫폼

---

## 패키지 구성

| 패키지 | 설명 |
|--------|------|
| `perception` | 카메라 구독, 차선 감지, 표지판 인식, OpenCV 시각화 |
| `simulation` | Gazebo 시뮬레이션 월드, 로봇 모델, ROS↔Gazebo 브리지 |

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
``

특정 패키지만 빌드하려면:

```bash
colcon build  --packages-select perception
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

## 시뮬레이션 실행

### 전체 시뮬레이션 한 번에 실행

```bash
ros2 launch simulation full_simulation.launch.py
```

Gazebo 월드, 로봇 스폰, ROS↔Gazebo 브리지, 인지 노드, OpenCV 시각화 창이 순서대로 실행됩니다.

### 실행 순서 (내부적으로 자동 처리)

| 시각 | 실행되는 것 |
|------|------------|
| t = 0s | Gazebo 월드 (`track.sdf`) 시작 |
| t = 0s | `robot_state_publisher` — URDF 로드 |
| t = 3s | 로봇 엔티티 스폰 (`hyper`) |
| t = 4s | `ros_gz_bridge` — 토픽 브리지 시작 |
| t = 5s | 인지 노드 + 시각화 창 시작 |

### 실행 옵션

```bash
# 월드 파일 변경
ros2 launch simulation full_simulation.launch.py world:=/path/to/custom.sdf

# 로봇 초기 위치 지정
ros2 launch simulation full_simulation.launch.py x:=2.0 y:=1.0 z:=0.1
```

---

## 인지 노드 개별 실행

시뮬레이션 없이 실제 카메라나 rosbag으로 테스트할 때:

```bash
# 카메라 구독 (로그 출력)
ros2 run perception camera_subscriber

# 차선 감지 → /lane/center 퍼블리시
ros2 run perception lane_detector

# 표지판 인식 → /perception/sign 퍼블리시
ros2 run perception sign_detector --ros-args -p model_path:=/path/to/best.pt

# OpenCV 시각화 창
ros2 run perception visualizer
```

---

## 주요 토픽

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/image_raw` | `sensor_msgs/Image` | Gazebo → ROS | 카메라 영상 |
| `/lane/center` | `std_msgs/Float64MultiArray` | perception → | `[offset, heading_err, valid]` |
| `/perception/sign` | `vision_msgs/Detection2DArray` | perception → | 표지판 감지 결과 |
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
