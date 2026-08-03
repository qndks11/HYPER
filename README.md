# HYPER

HL FMA 2026 1/5 — ROS 2 기반 자율주행 차량 플랫폼

---

## Command to run

### 한 번에 실행 — `hyper_launch`

```bash
ros2 launch hyper_launch simulation.launch.py
```

한 프로세스 트리 안에서 스택 전체를 순서대로 띄웁니다: `sim`(Gazebo + 차량 스폰 + 저수준 컨트롤러) → 5초 뒤 `odometry`(dual EKF + navsat_transform) → 7초 뒤 `perception`(차선/정지선 감지 + 신호등 감지) → 9초 뒤 `behavior`(`hyper_planner` 패키지의 `parking_system_cpp.launch.py` — costmap, hybrid A* 플래너, behavior supervisor, controller를 한 번에 실행하는 C++ 버전).

스택을 끄려면 `Ctrl-C` 한 번으로 전체 트리가 종료됩니다.

각 단계는 `src/launcher/hyper_launch/launch/`에 개별 launch 파일(`sim.launch.py`, `odometry.launch.py`, `perception.launch.py`, `behavior.launch.py`)로도 있어 단독 실행이 가능합니다:

```bash
ros2 launch hyper_launch sim.launch.py
ros2 launch hyper_launch odometry.launch.py
ros2 launch hyper_launch perception.launch.py
ros2 launch hyper_launch behavior.launch.py
```

## 패키지 구성

| 패키지 | 설명 |
|--------|------|
| `hyper_launch` | 전체 스택 launch (단계별 launch 파일 + 마스터 `simulation.launch.py`) |
| `hyper_control` | 차량 제어(Ackermann/조이스틱 텔레옵), Gazebo 시뮬레이션용 로봇 모델 |
| `hyper_gazebo` | Gazebo 시뮬레이션 전용 (월드, gz 플러그인), `hyper_control`에 의존 |
| `hyper_planner` | 행동/계획 스택 (behavior supervisor, controller) |
| `hyper_localization` | dual EKF + navsat_transform (robot_localization 래핑, GPS 융합 오도메트리) |
| `hyper_lane_detection` | 카메라 영상 기반 차선/정지선 감지 + OpenCV 디버그 대시보드 |
| `hyper_object_detection` | YOLO 기반 객체/신호등 감지 + OpenCV 디버그 대시보드 |
| `hyper_rtk` | u-blox GPS 드라이버 + NTRIP 클라이언트 실행 (RTK 보정 위치) |
| `hyper_camera` | 실차 USB 카메라 구동 + rectify (`usb_cam` + `image_proc` 컴포저블 파이프라인) |
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
colcon build --packages-select hyper_control
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

## GPS(RTK) 설치

`hyper_rtk`(`src/sensing/hyper_rtk`) 패키지가 u-blox GPS 드라이버(`ublox_gps`)와 NTRIP 클라이언트(`ntrip_client`)를 함께 실행해 RTK 보정 위치를 퍼블리시합니다. 두 드라이버는 rosdep으로 설치되지 않는 소스 패키지라 vcstool로 따로 받아야 합니다.

### 1. 드라이버 소스 받기

```bash
cd ~/HYPER
vcs import src < deps.repos
rosdep install --from-paths src --ignore-src -r -y
```

`src/sensing/ublox`, `src/sensing/ntrip_client`가 새로 생깁니다 (`.gitignore` 대상 — vcstool로만 관리, 저장소에는 커밋되지 않음).

### 2. GPS 장치 권한 / udev 규칙

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

### 3. NTRIP 계정 설정

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

### 4. 빌드 & 실행

```bash
colcon build --packages-select ublox_gps ublox_msgs ublox_serialization ntrip_client hyper_rtk
source ~/HYPER/install/setup.bash
ros2 launch hyper_rtk rtk.launch.py
```

정상 동작하면 `ublox_gps_node`가 `/ublox_gps_node/fix`(GPS 위치)를, `ntrip_client`가 NTRIP 캐스터에서 받은 RTCM 보정 데이터를 퍼블리시합니다.

---

## 카메라 설치

`hyper_camera`(`src/sensing/hyper_camera`) 패키지가 실차 USB 카메라(ELP-USBGS1200P01-KL170, global shutter)를 `usb_cam` 드라이버로 구동하고, 같은 컴포넌트 컨테이너 안에서 `image_proc`으로 rectify까지 처리합니다. `usb_cam`도 rosdep으로 설치되지 않는 소스 패키지라 GPS(RTK) 절의 `vcs import` 단계에서 `src/sensing/usb_cam`(우리 fork, `hyper-fixes` 브랜치)으로 함께 받아집니다.

### 1. 장치 udev 규칙

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

규칙 적용 후 `hyper_camera`의 `config/params_1.yaml`은 기본값으로 `/dev/video_elp`를 사용하므로, USB 포트가 바뀌어도 흔들리지 않습니다.

### 2. 빌드 & 실행

```bash
colcon build --packages-select usb_cam hyper_camera
source ~/HYPER/install/setup.bash
ros2 launch hyper_camera camera.launch.py
```

정상 동작하면 `/image_raw`(원본 영상), `/image_rect`(rectify된 영상), `/camera_info`(ELP-USBGS1200P01-KL170 캘리브레이션)가 퍼블리시됩니다.

---

## 주요 토픽

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo → ROS | 카메라 영상 (시뮬레이션) |
| `/image_raw` | `sensor_msgs/Image` | hyper_camera → | 실차 USB 카메라 원본 영상 (`usb_cam`) |
| `/image_rect` | `sensor_msgs/Image` | hyper_camera → | rectify된 영상 (`image_proc`) |
| `/camera_info` | `sensor_msgs/CameraInfo` | hyper_camera → | ELP-USBGS1200P01-KL170 캘리브레이션 |
| `/lane/center` | `std_msgs/Float64MultiArray` | hyper_lane_detection → | `[left_offset_m, left_steering_deg, left_valid, right_offset_m, right_steering_deg, right_valid]` |
| `/stopline/detection` | `std_msgs/Float64MultiArray` | hyper_lane_detection → | `[distance_m, valid]` |
| `/perception/sign` | `std_msgs/String` | hyper_object_detection → | `red` / `green` / `left_arrow` / `none` |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → Gazebo | 속도 명령 |
| `/odom` | `nav_msgs/Odometry` | Gazebo → ROS | 오도메트리 |
| `/imu` | `sensor_msgs/Imu` | Gazebo → ROS | IMU |

