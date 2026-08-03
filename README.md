# HYPER

HL FMA 2026 1/5 — ROS 2 기반 자율주행 차량 플랫폼

---

## 설치

### 1. Ubuntu 22.04

ROS 2 Humble은 Ubuntu 22.04 (Jammy)를 기준으로 배포되므로 먼저 [Ubuntu 22.04](https://releases.ubuntu.com/jammy/)를 설치합니다.

### 2. ROS 2 Humble 설치

[공식 설치 가이드](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)를 따라 `ros-humble-desktop`을 설치합니다. 요약:

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool

sudo rosdep init
rosdep update
```

터미널마다 ROS 2 환경을 불러와야 하므로 `~/.bashrc`에 추가해둡니다:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. 저장소 클론

```bash
git clone https://github.com/qndks11/HYPER.git ~/HYPER
cd ~/HYPER
```

### 4. 의존성 설치 (vcstool + rosdep)

`hyper_rtk`(u-blox, NTRIP)와 `hyper_camera`(usb_cam)가 쓰는 드라이버는 rosdep으로 설치되지 않는 소스 패키지라 `deps.repos`로 따로 받아야 합니다. RTK/카메라를 쓰지 않더라도 먼저 받아두면 이후 센서 설치 절을 그대로 따라갈 수 있습니다.

```bash
cd ~/HYPER
vcs import src < deps.repos
rosdep install --from-paths src --ignore-src -r -y
```

`src/sensing/ublox`, `src/sensing/ntrip_client`, `src/sensing/usb_cam`이 새로 생깁니다 (`.gitignore` 대상 — vcstool로만 관리, 저장소에는 커밋되지 않음).

### 5. 빌드 & 환경 소싱

```bash
colcon build
```

특정 패키지만 빌드하려면:

```bash
colcon build --packages-select hyper_control
```

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

## 센서 설치 (실차)

시뮬레이션만 쓴다면 이 절은 건너뛰어도 됩니다. 실차에서 GPS(RTK), LiDAR, USB 카메라를 쓰려면 아래를 순서대로 진행합니다.

### GPS (RTK)

`hyper_rtk`(`src/sensing/hyper_rtk`) 패키지가 u-blox GPS 드라이버(`ublox_gps`)와 NTRIP 클라이언트(`ntrip_client`)를 함께 실행해 RTK 보정 위치를 퍼블리시합니다.

#### 1. GPS 장치 권한 / udev 규칙

```bash
sudo usermod -aG dialout $USER   # 적용하려면 재로그인 필요
```

launch 파일(`rtk.launch.py`)이 장치를 고정 이름 `/dev/tty_Ardusimple`로 찾으므로, USB 포트 번호가 바뀌어도 안 흔들리도록 udev 규칙을 등록합니다. `/etc/udev/rules.d/99-ardusimple.rules` 생성:

```
KERNEL=="ttyACM[0-9]*", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK="tty_Ardusimple", GROUP="dialout", MODE="0666"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

다른 Ardusimple 보드/케이블을 쓴다면 `idVendor`/`idProduct`가 다를 수 있으니, 장치를 연결한 상태에서 아래로 직접 확인 후 값을 맞춰주세요:

```bash
udevadm info -a -n /dev/ttyACM0 | grep -E "idVendor|idProduct" | head -2
```

#### 2. NTRIP 계정 설정

`src/sensing/hyper_rtk/config/ntrip_params.yaml`은 NTRIP 캐스터 로그인 정보가 들어있어 `.gitignore` 대상입니다(저장소에는 없음). 없다면 새로 만듭니다:

```yaml
ntrip_client:
  ros__parameters:
    host: "<NTRIP 서버 주소>"       # 예: www.gnssdata.or.kr
    port: 2101
    mountpoint: "<마운트포인트>"     # 예: SOUL-RTCM32
    ntrip_version: "None"
    authenticate: true
    username: "<계정>"
    password: "<비밀번호>"
    ssl: false
    cert: "None"
    key: "None"
    ca_cert: "None"
    rtcm_frame_id: "odom"
    nmea_max_length: 128
    nmea_min_length: 3
    rtcm_message_package: "rtcm_msgs"
    reconnect_attempt_max: 10
    reconnect_attempt_wait_seconds: 5
    rtcm_timeout_seconds: 4
```

#### 3. 실행

```bash
ros2 launch hyper_rtk rtk.launch.py
```

정상 동작하면 `ublox_gps_node`가 `/gps/fix`(GPS 위치, `hyper_localization`의 `navsat_transform_node`가 구독하는 토픽과 동일)를, `ntrip_client`가 NTRIP 캐스터에서 받은 RTCM 보정 데이터를 퍼블리시합니다.

### LiDAR

`hyper_lidar`(`src/sensing/hyper_lidar`) 패키지가 실차에 연결된 RPLidar 등 2D LiDAR 드라이버(`rplidar_ros`, rosdep으로 설치됨)를 실행합니다. 시뮬레이션에서는 사용하지 않으며, Gazebo가 `/scan`을 직접 발행합니다.

장치 포트, 보드레이트, 프레임 이름은 `config/rplidar_params.yaml`에서 실제 장비에 맞게 설정합니다 (기본값은 `/dev/rplidar` — 포트가 바뀌어도 흔들리지 않도록 GPS/카메라와 마찬가지로 idVendor/idProduct 기준 udev 심볼릭 링크를 걸어두는 것을 권장합니다).

```bash
ros2 launch hyper_lidar rplidar.launch.py
```

### USB 카메라

전방 ELP USB 카메라(ELP-USBGS1200P01-KL170, global shutter)는 `hyper_lane_detection`이 `input_backend:=direct_usb`일 때 `/dev/video_elp`를 직접 열어 MJPEG을 캡처·디코드하고 자체적으로 rectify까지 처리합니다 (중간 ROS 이미지 토픽 없음) — 실차 전체/단계별 launch(`real.launch.py` → `sensors.launch.py` + `perception.launch.py`)는 기본적으로 이 경로를 씁니다.

`hyper_camera`(`src/sensing/hyper_camera`) 패키지는 이 카메라의 보정 파일(`config/ELP-USBGS1200P01-KL170.yaml`, `direct_usb`가 런타임에 참조)의 배포처로 남아 있고, `usb_cam` 드라이버 + `image_proc` rectify를 ROS 토픽으로 띄우는 독립 실행(아래) 및 롤백용 `input_backend:=ros_compressed` 경로에 계속 쓸 수 있습니다.

#### 1. 장치 udev 규칙

`video_device`가 `/dev/videoN` 고정 번호를 참조하는데, 다른 UVC 장치(내장/외장 웹캠 등)가 함께 연결되어 있으면 부팅·재연결 시 번호가 바뀔 수 있고, 최악의 경우 엉뚱한 카메라를 열어 ELP 캘리브레이션이 잘못된 영상에 적용됩니다. GPS와 동일하게 idVendor/idProduct 기준 udev 심볼릭 링크로 고정합니다. `/etc/udev/rules.d/99-elp-camera.rules` 생성:

```
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="32e4", ATTRS{idProduct}=="0234", ATTR{index}=="0", SYMLINK+="video_elp"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

다른 카메라 모듈을 쓴다면 idVendor/idProduct가 다를 수 있으니, 장치를 연결한 상태에서 아래로 직접 확인 후 값을 맞춰주세요:

```bash
udevadm info -a -n /dev/video2 | grep -E "idVendor|idProduct" | head -2
```

UVC 카메라는 보통 `/dev/videoN`을 두 개(캡처 노드 + 메타데이터 노드) 만드는데, `v4l2-ctl --list-devices`로 카메라 이름 아래 첫 번째로 뜨는 노드가 캡처 노드입니다(`ATTR{index}=="0"`). video4linux 장치는 로그인 세션에 대해 보통 자동으로 접근 권한(ACL)이 부여되므로, RTK의 `dialout` 그룹과 달리 별도 그룹 설정은 필요 없습니다.

규칙 적용 후 `hyper_camera`의 `config/params_1.yaml`과 `hyper_lane_detection`의 `input_backend:=direct_usb`(파라미터 `video_device`) 둘 다 기본값으로 `/dev/video_elp`를 사용하므로, USB 포트가 바뀌어도 흔들리지 않습니다.

#### 2. 독립 실행 (선택 — 기본 실차 경로에는 필요 없음)

```bash
ros2 launch hyper_camera camera.launch.py
```

정상 동작하면 `/image_raw`(원본 영상), `/image_rect`(rectify된 영상), `/camera_info`(ELP-USBGS1200P01-KL170 캘리브레이션)가 퍼블리시됩니다. 롤백용 `input_backend:=ros_compressed` 경로를 테스트하거나, 카메라 영상을 다른 용도로 ROS 토픽으로 띄워야 할 때만 이렇게 독립 실행하면 됩니다.

### RealSense D435i

`realsense2_camera`(rosdep으로 설치되는 apt 패키지, `sudo apt install ros-humble-realsense2-camera`)가 D435i를 구동합니다. 이 저장소에는 전용 패키지가 없고, `hyper_launch`의 `sensors.launch.py`가 `realsense2_camera`의 `rs_launch.py`를 직접 include해서 `camera_name:=camera_object`로 띄우고 객체 인식용 `/camera_object/image_raw`와 EKF용 `/imu`(`unite_imu_method:=1`로 합성된 gyro+accel)로 리매핑합니다.

```bash
ros2 launch realsense2_camera rs_launch.py camera_name:=camera_object camera_namespace:='' enable_gyro:=true enable_accel:=true unite_imu_method:=1
```

---

## Command to run

### 한 번에 실행 — `hyper_launch`

시뮬레이션:

```bash
ros2 launch hyper_launch simulation.launch.py
```

한 프로세스 트리 안에서 스택 전체를 순서대로 띄웁니다: `sim`(Gazebo + 차량 스폰 + 저수준 컨트롤러) → 5초 뒤 `odometry`(dual EKF + navsat_transform) → 7초 뒤 `perception`(차선/정지선 감지 + 신호등 감지) → 9초 뒤 `behavior`(`hyper_planner` 패키지의 `parking_system_cpp.launch.py` — costmap, hybrid A* 플래너, behavior supervisor, controller를 한 번에 실행하는 C++ 버전).

실차 (Gazebo 대신 실제 센서로 동일한 파이프라인):

```bash
ros2 launch hyper_launch real.launch.py
```

`sim` 대신 `sensors`(D435i + RPLidar + RTK)가 먼저 뜨고, 이후 `odometry`/`perception`/`behavior`는 시뮬레이션과 동일하게 staggered로 이어집니다. 전방 ELP 카메라는 `sensors`가 아니라 `perception` 단계에서 `hyper_lane_detection`이 `input_backend:=direct_usb`로 직접 열므로, `sensors.launch.py`에는 더 이상 포함되지 않습니다.

스택을 끄려면 `Ctrl-C` 한 번으로 전체 트리가 종료됩니다.

각 단계는 `src/launcher/hyper_launch/launch/`에 개별 launch 파일로도 있어 단독 실행이 가능합니다:

```bash
ros2 launch hyper_launch sim.launch.py          # 시뮬레이션만 (실차: sensors.launch.py)
ros2 launch hyper_launch sensors.launch.py      # 실차 센서만
ros2 launch hyper_launch odometry.launch.py
ros2 launch hyper_launch perception.launch.py
ros2 launch hyper_launch behavior.launch.py
```

#### 실차 센서 한 번에 실행 — `sensors.launch.py`

물리 센서 3개를 각각 `hyper_localization`이 기대하는 토픽 역할로 리매핑해서 묶어 띄웁니다:

| 센서 | 패키지 | 최종 토픽 |
|------|--------|-----------|
| RealSense D435i | `realsense2_camera` | `/camera_object/image_raw` (객체 인식), `/imu` (EKF) |
| RPLidar | `hyper_lidar` | `/scan` |
| u-blox + NTRIP | `hyper_rtk` | `/gps/fix` |

전방 ELP usb_cam은 여기 포함되지 않습니다 — `perception.launch.py`가 `hyper_lane_detection`을 `input_backend:=direct_usb`로 실행하면서 `/dev/video_elp`를 직접 열고 자체 rectify까지 수행하므로, 이 카메라를 위한 별도 ROS 이미지 토픽/노드가 없습니다.

후방 카메라(`/camera_rear/image_raw`)는 아직 실물이 없고, `direct_usb`는 애초에 후방 경로가 없습니다 — `hyper_lane_detection`은 후방 프레임 없이도 동작해야 합니다.

## 패키지 구성

| 패키지 | 설명 |
|--------|------|
| `hyper_launch` | 전체 스택 launch (단계별 launch 파일 + 마스터 `simulation.launch.py`/`real.launch.py`) |
| `hyper_control` | 차량 제어(Ackermann/조이스틱 텔레옵), Gazebo 시뮬레이션용 로봇 모델 |
| `hyper_gazebo` | Gazebo 시뮬레이션 전용 (월드, gz 플러그인), `hyper_control`에 의존 |
| `hyper_planner` | 행동/계획 스택 (behavior supervisor, controller) |
| `hyper_localization` | dual EKF + navsat_transform (robot_localization 래핑, GPS 융합 오도메트리) |
| `hyper_lane_detection` | 카메라 영상 기반 차선/정지선 감지 + OpenCV 디버그 대시보드 |
| `hyper_object_detection` | YOLO 기반 객체/신호등 감지 + OpenCV 디버그 대시보드 |
| `hyper_rtk` | u-blox GPS 드라이버 + NTRIP 클라이언트 실행 (RTK 보정 위치) |
| `hyper_lidar` | 실차 RPLidar 등 2D LiDAR 드라이버 실행 (시뮬레이션에서는 미사용) |
| `hyper_camera` | 실차 ELP USB 카메라 보정 파일 배포처 + 독립 실행용 `usb_cam`/`image_proc` 파이프라인 (기본 `direct_usb` 경로에서는 실행되지 않음, 롤백/단독 실행용) |
---

## 주요 토픽

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo → ROS (시뮬레이션만) | 전방 카메라 영상 (차선 인식, `input_backend:=ros_raw`가 구독). 실차 기본 경로(`direct_usb`)에서는 이 토픽 자체가 존재하지 않음 — `hyper_lane_detection`이 카메라를 직접 열고 끝까지 인메모리로 처리 |
| `/camera_object/image_raw` | `sensor_msgs/Image` | Gazebo → ROS (시뮬레이션) / realsense2_camera → ROS (실차, D435i) | 객체 인식용 카메라 영상 |
| `/camera_rear/image_raw` | `sensor_msgs/Image` | Gazebo → ROS | 후방 카메라 영상 (시뮬레이션만 — 실차 후방 카메라 미장착, `direct_usb`는 후방 경로 자체가 없음) |
| `/image_raw` | `sensor_msgs/Image` | hyper_camera → | `hyper_camera camera.launch.py` 단독 실행(롤백/테스트용) 시 실차 USB 카메라 원본 영상 (`usb_cam`, 리매핑 전) — 기본 `direct_usb` 경로에서는 발행되지 않음 |
| `/image_rect` | `sensor_msgs/Image` | hyper_camera → | rectify된 영상 (`image_proc`, 위와 동일하게 단독 실행 시에만) |
| `/camera_info` | `sensor_msgs/CameraInfo` | hyper_camera → | ELP-USBGS1200P01-KL170 캘리브레이션 (단독 실행 시에만; `direct_usb`는 같은 yaml을 `lane_detection_node`가 직접 읽음) |
| `/scan` | `sensor_msgs/LaserScan` | hyper_lidar / Gazebo → | 2D LiDAR 스캔 (실차: RPLidar, 시뮬레이션: Gazebo) |
| `/gps/fix` | `sensor_msgs/NavSatFix` | Gazebo → ROS (시뮬레이션) / hyper_rtk → ROS (실차) | GPS 위경도 (navsat_transform 입력) |
| `/lane/center` | `std_msgs/Float64MultiArray` | hyper_lane_detection → | `[left_offset_m, left_steering_deg, left_valid, right_offset_m, right_steering_deg, right_valid]` |
| `/stopline/detection` | `std_msgs/Float64MultiArray` | hyper_lane_detection → | `[distance_m, valid]` |
| `/perception/sign` | `std_msgs/String` | hyper_object_detection → | `red` / `green` / `left_arrow` / `none` |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → Gazebo | 속도 명령 |
| `/odom` | `nav_msgs/Odometry` | Gazebo → ROS | 오도메트리 |
| `/imu` | `sensor_msgs/Imu` | Gazebo → ROS (시뮬레이션) / realsense2_camera → ROS (실차, D435i 내장 IMU) | IMU |
