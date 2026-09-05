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

`hyper_rtk`(u-blox, NTRIP)가 쓰는 드라이버는 rosdep으로 설치되지 않는 소스 패키지라 `deps.repos`로 따로 받아야 합니다. RTK를 쓰지 않더라도 먼저 받아두면 이후 센서 설치 절을 그대로 따라갈 수 있습니다. (카메라 두 대는 모두 `hyper_camera`의 전용 드라이버 노드가 직접 여므로 `usb_cam` 드라이버가 더 이상 필요 없습니다 — 이 저장소에서 완전히 제거되었습니다.)

```bash
cd ~/HYPER
vcs import src < deps.repos
rosdep install --from-paths src --ignore-src -r -y
```

`src/sensing/ublox`, `src/sensing/ntrip_client`이 새로 생깁니다 (`.gitignore` 대상 — vcstool로만 관리, 저장소에는 커밋되지 않음).

### 5. pip 의존성 설치

`ultralytics`(YOLO, `hyper_object_detection`)는 rosdep으로 해석되지 않는 순수 pip 패키지라 별도로 설치해야 합니다. 저장소 루트의 `requirements.txt`에 정리되어 있습니다:

```bash
pip install -r ~/HYPER/requirements.txt
```

(`cv2`는 `cv_bridge`를 통해 rosdep이 이미 설치하므로 여기 포함되지 않습니다.)

`requirements.txt`는 `numpy<2`도 함께 고정합니다 — `ultralytics`를 제약 없이 설치하면 numpy 2.x로 끌어올릴 수 있는데, Humble의 `cv_bridge`는 apt `python3-numpy`(1.21.x) 기준 ABI로 빌드되어 있어 numpy 2.x가 깔리면 `cv2`/`cv_bridge` 임포트가 깨집니다. 이미 numpy 2.x가 설치되어 있다면 `pip install -r requirements.txt`로 다시 1.x대로 내려주세요.

### 6. 빌드 & 환경 소싱

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

시뮬레이션만 쓴다면 이 절은 건너뛰어도 됩니다. 실차에서 GPS(RTK), IMU, LiDAR, USB 카메라를 쓰려면 아래를 순서대로 진행합니다.

### USB 시리얼 포트 고정 (udev)

차에는 USB 시리얼 장치가 넷 물려 있습니다: RTK 수신기 2대(moving-base 헤딩용 base+rover), IMU(EBIMU) USB-UART 어댑터, Arduino 제어 보드. `/dev/ttyUSB*` 번호는 **꽂힌 순서로 정해지므로** 그대로 두면 부팅할 때마다 서로의 포트를 집습니다. 저장소의 `udev/99-hyper-serial.rules`가 네 장치에 고정 이름을 붙입니다:

| 심볼릭 링크 | 장치 | 쓰는 곳 |
|---|---|---|
| `/dev/tty_ublox_base` | Ardusimple simpleRTK2B (u-blox ZED-F9P), base — 뒤쪽 안테나 | `hyper_rtk/launch/rtk.launch.py` |
| `/dev/tty_ublox_rover` | Ardusimple simpleRTK2B (u-blox ZED-F9P), rover — 앞쪽 안테나 | `hyper_rtk/launch/rtk.launch.py` |
| `/dev/tty_ebimu` | EBIMU-9DOFV5 USB-UART 어댑터 | `hyper_ebimu/config/ebimu.yaml` |
| `/dev/tty_arduino` | Arduino 제어 보드 (CH340) | `hyper_interface/config/parameters.yaml` |

두 RTK 보드는 idVendor/idProduct까지 완전히 같지만(`1546:01a9`), 이 차의 두 보드에는 USB 시리얼 문자열이 이미 Flash에 기록돼 있어(`HYPER-GNSS-BASE` / `HYPER-GNSS-ROVER`) 규칙이 `ATTRS{serial}`로 바로 갈라냅니다. 물리 USB 포트(`KERNELS`)를 뽑아 채워 넣을 필요가 없고, 어느 포트에 꽂아도 링크 이름이 그대로 붙습니다.

```bash
sudo usermod -aG dialout $USER   # 적용하려면 재로그인 필요
sudo cp ~/HYPER/udev/99-hyper-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/tty_ublox_base /dev/tty_ublox_rover /dev/tty_ebimu /dev/tty_arduino
```

네 링크가 다 보이면 끝입니다. 안 보이면 장치의 실제 칩 ID(또는 시리얼 번호)가 규칙과 다른 것이므로, 지금 꽂혀 있는 장치들의 값을 뽑아서 규칙 파일을 고칩니다:

```bash
~/HYPER/udev/show-serial-ids.sh
```

### GPS (RTK) — 듀얼 GNSS moving-base 헤딩

`hyper_rtk`(`src/sensing/hyper_rtk`) 패키지가 u-blox ZED-F9P 보드 2대(base+rover)의 GPS 드라이버(`ublox_gps`)와 NTRIP 클라이언트(`ntrip_client`)를 함께 실행합니다. base가 NTRIP RTK로 절대 위치(`/gps/fix`)를 내고, rover는 base에서 UART2로 받은 보정으로 두 안테나 사이 상대위치를 풀어 절대 헤딩(`imu/heading`)을 낸다 — moving-base RTK 헤딩. 자세한 배선/u-center 설정은 `src/sensing/hyper_rtk/README.md` 참고.

#### 1. 안테나/배선

- 안테나 2개를 차량 세로축에 강체로 고정, 뒤=base·앞=rover (관례)
- base의 UART2 `TXD2` → rover의 UART2 `RXD2`, `GND` 공통
- u-center로 base UART2에 RTCM3(1077/1087/1097/1127/1230) + **4072.0**을, rover UART2 input + `UBX-NAV-RELPOSNED`를 켜고 플래시에 저장 (드라이버가 moving-base 경로에서 이 설정을 자동으로 안 밀어줌)

#### 2. GPS 장치 권한 / udev 규칙

위 [USB 시리얼 포트 고정](#usb-시리얼-포트-고정-udev--먼저-하세요)에서 규칙을 이미 깔았다면 `/dev/tty_ublox_base`/`/dev/tty_ublox_rover`가 만들어져 있고, `rtk.launch.py`가 그 이름으로 장치를 찾습니다. 확인:

```bash
ls -l /dev/tty_ublox_base /dev/tty_ublox_rover
```

두 보드는 idVendor/idProduct가 같으므로 규칙은 보드 Flash에 써 넣은 USB 시리얼 문자열(`HYPER-GNSS-BASE` / `HYPER-GNSS-ROVER`)로 갈라냅니다. 링크가 없으면 시리얼이 안 들어간 것이니 확인하고 다시 써 넣습니다:

```bash
~/HYPER/udev/ublox-serial.py --list                                # 칩 ID / 현재 시리얼
~/HYPER/udev/ublox-serial.py /dev/ttyACM0 HYPER-GNSS-ROVER --reset  # 다시 쓰기
```

#### 3. NTRIP 계정 설정 (base 전용)

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

#### 4. 실행

```bash
ros2 launch hyper_rtk rtk.launch.py
```

정상 동작하면 `ublox_gps_node_base`가 `/gps/fix`(GPS 위치, `hyper_localization`의 `navsat_transform_node`가 구독하는 토픽과 동일)를, `ntrip_client`가 NTRIP 캐스터에서 받은 RTCM 보정 데이터를 퍼블리시합니다. `ublox_gps_node_rover`는 `imu/heading`(`sensor_msgs/Imu`, yaw만 유효)을 냅니다 — `ekf_global`이 이걸 절대 방위로 씁니다(아래 IMU 절 참고).

```bash
ros2 topic echo /imu/heading                        # 헤딩 (orientation.z/w, yaw만 유효)
ros2 topic echo /ublox_gps_node_rover/navrelposned  # relPosHeading 원본 확인용
```

### IMU (EBIMU-9DOFV5)

`hyper_ebimu`(`src/sensing/hyper_ebimu`)가 E2BOX EBIMU-9DOFV5 AHRS 모듈의 UART/ASCII 프로토콜을 직접 파싱해 `/imu`(`sensor_msgs/Imu`, `body_link` 프레임)로 퍼블리시합니다. 모듈에 USB가 없어 USB-UART 어댑터로 물립니다 — 배선표와 센서 설정 명령은 `src/sensing/hyper_ebimu/README.md` 참고.

```bash
sudo apt install python3-serial     # 또는 pip install pyserial
ros2 launch hyper_ebimu ebimu.launch.py
ros2 topic hz /imu                  # 100 Hz 근처 (output_rate_ms=10)
```

포트는 `config/ebimu.yaml`의 `port`이고 기본값은 udev 규칙이 만드는 `/dev/tty_ebimu`입니다.

**절대 방위(yaw)는 이제 이 IMU가 아니라 위 GPS(RTK) 절의 듀얼 GNSS moving-base 헤딩이 담당합니다**(`dual_ekf_navsat.yaml`의 `ekf_global`, `imu0_config` 인덱스 5 = false / `imu1`(`imu/heading`) 인덱스 5 = true). EBIMU는 roll/pitch(중력 관측)와 자이로 z(연속 odom용)만 여전히 씁니다. 9축 AHRS의 지자기 yaw 자체는 계속 나오므로, GNSS 헤딩 하드웨어가 빠지면 `imu0_config`/`imu1`을 되돌려 폴백으로 쓸 수 있습니다 — 그럴 때는 아래처럼 축 방향을 실차에서 반드시 확인해야 합니다(드라이버는 센서 body frame을 재매핑 없이 내보내므로 REP-103 ENU(0=East, 반시계 +)와 어긋나면 map heading이 통째로 돌아갑니다):

```bash
ros2 topic echo /imu --field angular_velocity   # 반시계로 돌릴 때 z > 0
```

### LiDAR

`hyper_lidar`(`src/sensing/hyper_lidar`) 패키지가 실차에 연결된 RPLidar 등 2D LiDAR 드라이버(`rplidar_ros`, rosdep으로 설치됨)를 실행합니다. 시뮬레이션에서는 사용하지 않으며, Gazebo가 `/scan`을 직접 발행합니다.

장치 포트, 보드레이트, 프레임 이름은 `config/rplidar_params.yaml`에서 실제 장비에 맞게 설정합니다 (기본값은 `/dev/rplidar` — 포트가 바뀌어도 흔들리지 않도록 GPS/카메라와 마찬가지로 idVendor/idProduct 기준 udev 심볼릭 링크를 걸어두는 것을 권장합니다).

```bash
ros2 launch hyper_lidar rplidar.launch.py
```

### USB 카메라 (Logitech C920)

차량 카메라는 C920 **한 대**이고, 차선 인식(BEV)과 객체 인식(YOLO)이 같이 씁니다. `hyper_camera`의 `LogitechCameraPublisherNode`(C++ 컴포넌트, 실행 파일 `logitech_camera_publisher_node`)가 `/dev/video_logitech`를 열어 MJPEG을 640x360@30으로 캡처·디코드하고, rectify 없이 원본 프레임을 그대로 `image_raw`로 발행합니다 — 보정 파일이 없고(일반 약 70도 렌즈), `hyper_lane_detection`의 BEV 호모그래피가 이 카메라를 이상적인 핀홀로 모델링합니다(`config/bev_real.yaml`).

`hyper_lane_detection`이 `input_backend:=intra_process`일 때 이 컴포넌트가 `lane_detection`과 같은 `ComposableNodeContainer`에 함께 로드되어 프레임이 직렬화 없이 포인터로 넘어갑니다(zero-copy intra-process). 같은 발행이 DDS로도 나가므로 별도 프로세스인 `object_detection_node`(rclpy — zero-copy 경로 없음)가 동일한 프레임을 일반 토픽으로 받습니다. 실차 launch(`real.launch.py` → `perception.launch.py`)는 기본적으로 이 경로를 쓰고, `perception.launch.py` 자체의 기본값은 `ros_raw`(ros_gz_bridge 구독, 시뮬레이션/롤백용)입니다. `usb_cam`은 이 저장소에서 완전히 제거되었습니다.

#### 1. 장치 udev 규칙

`/dev/videoN` 번호는 다른 UVC 장치(내장/외장 웹캠 등)가 함께 물려 있으면 부팅·재연결 때마다 바뀌므로, 카메라도 시리얼 장치와 같은 규칙 파일에서 심볼릭 링크로 고정합니다 — 위 [USB 시리얼 포트 고정](#usb-시리얼-포트-고정-udev--먼저-하세요)에서 `udev/99-hyper-serial.rules`를 이미 깔았다면 `/dev/video_logitech`가 만들어져 있습니다:

```bash
ls -l /dev/video_logitech
```

UVC 카메라는 보통 `/dev/videoN`을 두 개(캡처 노드 + 메타데이터 노드) 만드는데, `v4l2-ctl --list-devices`로 카메라 이름 아래 첫 번째로 뜨는 노드가 캡처 노드입니다(규칙의 `ATTR{index}=="0"`). video4linux 장치는 로그인 세션에 대해 보통 자동으로 접근 권한(ACL)이 부여되므로, 시리얼 장치의 `dialout` 그룹과 달리 별도 그룹 설정은 필요 없습니다.

다른 카메라 모델로 바꾸면 idVendor/idProduct가 달라지니 확인해서 규칙을 고칩니다:

```bash
lsusb
udevadm info -a -n /dev/video0 | grep -E "idVendor|idProduct" | head -2
```

해상도·프레임레이트는 노드의 `image_width`/`image_height`/`framerate` 파라미터로 조정합니다(기본값은 `hyper_camera`의 `config/params_logitech.yaml`과 동일). **캡처 해상도를 바꾸면 `hyper_lane_detection/config/bev_real.yaml`도 같이 고쳐야 합니다** — 가로 폭에서 초점거리를, 세로 높이에서 BEV 근거리 경계를 유도하기 때문입니다.

## 차량 제어 보드 설치 (실차)

`hyper_interface`의 `arduino_interface_node`가 `/velocity`[m/s], `/steering_angle`[rad] 명령 토픽을 그대로 구독해서 시리얼로 Arduino 보드에 전달하고, Arduino(`hyper_motor_interface.ino`)가 BTS7960(구동)/L298N(조향) 모터드라이버를 닫힌 루프로 구동합니다. `ros2_control` 경유 없이 명령 토픽 → Arduino로 직접 연결되는 구조입니다. 상세 배선/프로토콜/파라미터는 `src/interface/hyper_interface/README.md` 참고.

### 1. 장치 확인 및 의존성

```bash
sudo usermod -aG dialout $USER   # 적용하려면 재로그인 필요
sudo apt install python3-serial  # 또는 pip install pyserial
```

포트는 udev 규칙이 만드는 `/dev/tty_arduino`입니다([USB 시리얼 포트 고정](#usb-시리얼-포트-고정-udev--먼저-하세요) 참고). 이 보드는 CH340 칩이라 `/dev/ttyUSB*`로 잡히는데, IMU 어댑터(CP210x)도 같은 `/dev/ttyUSB*` 번호를 나눠 쓰므로 번호로 잡으면 안 됩니다:

```bash
ls -l /dev/tty_arduino
```

### 2. 펌웨어 업로드

Arduino IDE로 `src/interface/hyper_interface/arduino/hyper_motor_interface/hyper_motor_interface.ino`를 보드에 업로드합니다.

### 3. 실행

```bash
ros2 launch hyper_interface interface.launch.py     # 기본 포트 /dev/tty_arduino
```

`real.launch.py`가 기본으로 이걸 포함하므로(behavior 단계와 함께 시작) 보통 따로 실행할 필요는 없습니다. `hyper_control`/`hyper_planner`의 `max_velocity`/`max_steering_angle`과 `hyper_interface/config/parameters.yaml`의 동일 파라미터가 어긋나면 Arduino가 튜닝 범위 밖의 명령을 받을 수 있으니 값을 맞춰둡니다.

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

`sim` 대신 `sensors`(EBIMU-9DOFV5 IMU + RPLidar + RTK)가 먼저 뜨고, 이후 `odometry`/`perception`/`behavior`는 시뮬레이션과 동일하게 staggered로 이어집니다. 카메라(Logitech C920)는 `sensors`가 아니라 `perception` 단계에서 열리므로 `sensors.launch.py`에는 포함되지 않습니다.

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
| E2BOX EBIMU-9DOFV5 | `hyper_ebimu` | `/imu` (EKF roll/pitch + gyro) |
| RPLidar | `hyper_lidar` | `/scan` |
| u-blox base + NTRIP | `hyper_rtk` | `/gps/fix` |
| u-blox rover (moving-base) | `hyper_rtk` | `/imu/heading` (EKF 절대 yaw) |

카메라는 여기 포함되지 않습니다 — `perception.launch.py`가 `lane_detection_container` 하나에 카메라 드라이버와 `lane_detection`을 함께 로드하고, `object_detection_node`는 같은 발행을 별도 프로세스에서 구독합니다. 카메라는 한 대이고 두 인지 노드가 같은 `/camera/image_raw`를 먹습니다:

| 소비자 | 노드 | 토픽 | 전달 방식 |
|--------|------|------|-----------|
| 차선 인식 | `hyper_camera`의 `LogitechCameraPublisherNode` + `lane_detection` (한 컨테이너) | `/camera/image_raw` | intra-process (zero-copy) |
| 객체 인식 | `hyper_object_detection`의 `object_detection_node` (별도 프로세스) | `/camera/image_raw` | 일반 토픽 (rclpy에는 zero-copy 경로 없음) |

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
| `hyper_rtk` | u-blox GPS 드라이버 두 대(moving base) + NTRIP 클라이언트 실행 (RTK 보정 위치 + 절대 방위) |
| `hyper_lidar` | 실차 RPLidar 등 2D LiDAR 드라이버 실행 (시뮬레이션에서는 미사용) |
| `hyper_camera` | 실차 Logitech C920 USB 카메라 드라이버 노드(`LogitechCameraPublisherNode` C++ 컴포넌트) + 참고용 파라미터 yaml 배포처 — `usb_cam`은 이 저장소에서 완전히 제거됨 |
| `hyper_interface` | 실차 ROS 2 ↔ Arduino 시리얼 브릿지 (`arduino_interface_node`) — `/velocity`, `/steering_angle`을 구독해 Arduino(`hyper_motor_interface.ino`)로 전달, BTS7960/L298N 모터드라이버를 닫힌 루프로 구동 |
---

## 주요 토픽

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo → ROS (시뮬레이션) / hyper_camera → lane_detection + object_detection_node (실차) | 전방 카메라 영상 — 차량의 유일한 카메라이고 차선·객체 인식이 같이 구독합니다. 시뮬레이션은 ros_gz_bridge, 실차는 `hyper_camera`의 `LogitechCameraPublisherNode`가 발행 — 실차 기본 경로(`lane_input_backend:=intra_process`)에서 `lane_detection`은 같은 `ComposableNodeContainer` 안에서 zero-copy로 받고, 같은 발행이 DDS로도 나가 별도 프로세스인 `object_detection_node`가 일반 구독으로 받습니다 |
| `/scan` | `sensor_msgs/LaserScan` | hyper_lidar / Gazebo → | 2D LiDAR 스캔 (실차: RPLidar, 시뮬레이션: Gazebo) |
| `/gps/fix` | `sensor_msgs/NavSatFix` | Gazebo → ROS (시뮬레이션) / hyper_rtk → ROS (실차) | GPS 위경도 (navsat_transform 입력) |
| `/lane/center` | `std_msgs/Float64MultiArray` | hyper_lane_detection → | `[left_offset_m, left_steering_deg, left_valid, right_offset_m, right_steering_deg, right_valid]` |
| `/stopline/detection` | `std_msgs/Float64MultiArray` | hyper_lane_detection → | `[distance_m, valid]` |
| `/perception/sign` | `std_msgs/String` | hyper_object_detection → | `red` / `green` / `left_arrow` / `none` |
| `/steering_angle` | `std_msgs/Float64` | → vehicle_controller_node (시뮬레이션) / arduino_interface_node (실차) | 목표 조향각 [rad] (teleop / planner / waypoint_tracker가 발행) |
| `/velocity` | `std_msgs/Float64` | → vehicle_controller_node (시뮬레이션) / arduino_interface_node (실차) | 목표 속도 [m/s] (teleop / planner / waypoint_tracker가 발행) |
| `/forward_position_controller/commands` | `std_msgs/Float64MultiArray` | vehicle_controller_node → ros2_control | 좌우 앞바퀴 조향각 (Ackermann 변환, 시뮬레이션 전용 — Gazebo의 `gz_ros2_control` 플러그인이 소비) |
| `/forward_velocity_controller/commands` | `std_msgs/Float64MultiArray` | vehicle_controller_node → ros2_control | 좌우 뒷바퀴 각속도 (차동 변환, 시뮬레이션 전용 — Gazebo의 `gz_ros2_control` 플러그인이 소비) |
| `/velocity_actual`, `/steering_angle_actual` | `std_msgs/Float64` | arduino_interface_node → | Arduino의 인코더/조향각 센서로 측정한 실제 값 (실차 전용, 닫힌 루프 피드백) |
| `/odom` | `nav_msgs/Odometry` | Gazebo → ROS (시뮬레이션) / arduino_interface_node → ROS (실차, bicycle-model dead reckoning) | 오도메트리 |
| `/imu` | `sensor_msgs/Imu` | Gazebo → ROS (시뮬레이션) / `hyper_ebimu` → ROS (실차, EBIMU-9DOFV5) | IMU. roll/pitch(중력 관측)와 자이로 z를 `ekf_global`/`ekf_local`이 씀. yaw는 더 이상 절대 방위로 쓰지 않음(`imu0_config` 인덱스 5 = false) — 아래 `/imu/heading` 참고. 실차 축이 REP-103 ENU와 어긋나면 `odometry.launch.py`의 `imu_enu_relay` 주석을 참고해 보정 |
| `/imu/heading` | `sensor_msgs/Imu` (yaw만 유효) | `ublox_gps_node_rover`(`hyper_rtk`) → `ekf_global` | 듀얼 GNSS moving-base RTK 헤딩(NAV-RELPOSNED9의 relPosHeading을 드라이버가 직접 ENU로 변환). `ekf_global`의 절대 방위 기준(`imu1`, 인덱스 5 = true). 정지 상태에서도 유효, 지자기 교란 영향 없음. EBIMU 지자기 yaw로 되돌리려면 `dual_ekf_navsat.yaml`에서 `imu0_config` 인덱스 5를 true로, `imu1` 블록을 지운다 |
