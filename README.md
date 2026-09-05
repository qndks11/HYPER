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

### USB 시리얼 포트 고정 (udev) — 먼저 하세요

차에는 USB 시리얼 장치가 넷 물려 있습니다: RTK 수신기 **두 대**(moving base + rover), IMU(EBIMU) USB-UART 어댑터, Arduino 제어 보드. `/dev/ttyUSB*` 번호는 **꽂힌 순서로 정해지므로** 그대로 두면 부팅할 때마다 IMU와 Arduino가 서로의 포트를 집습니다. 저장소의 `udev/99-hyper-serial.rules`가 각 장치에 고정 이름을 붙입니다:

| 심볼릭 링크 | 장치 | 쓰는 곳 |
|---|---|---|
| `/dev/tty_f9p_base` | Ardusimple simpleRTK2B #1 (u-blox ZED-F9P, moving base) | `hyper_rtk/config/f9p_base.yaml` |
| `/dev/tty_f9p_rover` | Ardusimple simpleRTK2B #2 (u-blox ZED-F9P, heading rover) | `hyper_rtk/config/f9p_rover.yaml` |
| `/dev/tty_ebimu` | EBIMU-9DOFV5 USB-UART 어댑터 | `hyper_ebimu/config/ebimu.yaml` |
| `/dev/tty_arduino` | Arduino 제어 보드 (CH340) | `hyper_interface/config/parameters.yaml` |

```bash
sudo usermod -aG dialout $USER   # 적용하려면 재로그인 필요
sudo cp ~/HYPER/udev/99-hyper-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/tty_f9p_base /dev/tty_f9p_rover /dev/tty_ebimu /dev/tty_arduino
```

네 링크가 다 보이면 끝입니다. 안 보이면 장치의 실제 칩 ID가 규칙과 다른 것이므로, 지금 꽂혀 있는 장치들의 값을 뽑아서 규칙 파일을 고칩니다:

```bash
~/HYPER/udev/show-serial-ids.sh
```

**주의**: F9P 두 대는 VID:PID가 `1546:01a9`로 **완전히 같아서** 칩 ID만으로는 절대 구분되지 않습니다. 반드시 `ATTRS{serial}`이나 물리 USB 포트(`KERNELS`)로 갈라야 하며, 규칙 파일에 두 방법이 다 적혀 있습니다. 잘못 배정하면 heading이 조용히 180도 뒤집힙니다.

같은 함정이 하나 더 있습니다: IMU 어댑터와 Arduino 보드가 둘 다 CH340(`1a86:7523`)이면 칩 ID만으로는 구분되지 않아 심볼릭 링크가 한쪽으로 덮입니다. 이때는 `ATTRS{serial}`이나 물리 USB 포트(`KERNELS`)를 같이 걸어야 하며, 방법은 규칙 파일 상단 주석에 적어 뒀습니다.

### GPS (RTK)

`hyper_rtk`(`src/sensing/hyper_rtk`) 패키지가 u-blox GPS 드라이버(`ublox_gps`) **두 개**와 NTRIP 클라이언트(`ntrip_client`)를 함께 실행해, RTK 보정 **위치**(`/gps/fix`)와 안테나 기선 기반 절대 **방위**(`/imu/heading`)를 함께 퍼블리시합니다. 배선도와 u-center에서 미리 해 둬야 할 설정은 `src/sensing/hyper_rtk/README.md`에 있습니다.

#### 1. GPS 장치 권한 / udev 규칙

위 [USB 시리얼 포트 고정](#usb-시리얼-포트-고정-udev--먼저-하세요)에서 규칙을 이미 깔았다면 두 링크가 만들어져 있고, `f9p_base.yaml` / `f9p_rover.yaml`이 그 이름으로 장치를 찾습니다. 확인:

```bash
ls -l /dev/tty_f9p_base /dev/tty_f9p_rover
```

다른 Ardusimple 보드/케이블을 쓰면 `idVendor`/`idProduct`가 다를 수 있으니, 링크가 없으면 장치를 연결한 상태에서 값을 확인해 `udev/99-hyper-serial.rules`를 고칩니다:

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
ros2 launch hyper_rtk dual_rtk.launch.py
```

정상 동작하면 base 쪽 `ublox_gps_node`가 `/gps/fix`(GPS 위치, `hyper_localization`의 `navsat_transform_node`가 구독하는 토픽과 동일)를, rover 쪽이 `/f9p_rover/navrelposned`(안테나 기선)를, `rtk_heading`이 `/imu/heading`(절대 방위)을, `ntrip_client`가 NTRIP 캐스터에서 받은 RTCM 보정 데이터를 퍼블리시합니다. `/imu/heading`이 안 나오면 RTK가 FIXED가 아닌 것이고, `rtk_heading`이 2초마다 기각 사유를 로그로 뱉습니다.

단일 수신기(위치만, 방위 없음)로 돌려야 하면 `rtk.launch.py`가 그대로 남아 있습니다.

### IMU (EBIMU-9DOFV5)

`hyper_ebimu`(`src/sensing/hyper_ebimu`)가 E2BOX EBIMU-9DOFV5 AHRS 모듈의 UART/ASCII 프로토콜을 직접 파싱해 `/imu`(`sensor_msgs/Imu`, `body_link` 프레임)로 퍼블리시합니다. 모듈에 USB가 없어 USB-UART 어댑터로 물립니다 — 배선표와 센서 설정 명령은 `src/sensing/hyper_ebimu/README.md` 참고.

```bash
sudo apt install python3-serial     # 또는 pip install pyserial
ros2 launch hyper_ebimu ebimu.launch.py
ros2 topic hz /imu                  # 100 Hz 근처 (output_rate_ms=10)
```

포트는 `config/ebimu.yaml`의 `port`이고 기본값은 udev 규칙이 만드는 `/dev/tty_ebimu`입니다.

9축(지자기 융합) AHRS라 켜는 순간부터 절대 방위가 나오지만, **이 yaw는 EKF가 쓰지 않습니다**(`dual_ekf_navsat.yaml`의 `imu0_config` 인덱스 5가 양쪽 필터 다 `false`). 실차에서 yaw가 ENU와 거울상이고 원점이 지자기 캘리브레이션 상태에 묶여 있는 것으로 확인돼(`datums.yaml` 주석), 절대 방위는 F9P 두 대(moving base)의 안테나 기선을 쓰는 `rtk_heading` 노드가 공급합니다(`/imu/heading` → `ekf_global`의 `imu1`, `hyper_rtk`). 여기서 EKF로 가는 건 roll/pitch, 각속도, 선가속도입니다.

드라이버는 센서 body frame을 재매핑 없이 내보내므로 **자이로 z 부호는 실차에서 확인해야 합니다** — 두 EKF 모두 `vyaw`를 먹습니다:

```bash
ros2 topic echo /imu --field angular_velocity   # 반시계로 돌릴 때 z > 0
```

### LiDAR

`hyper_lidar`(`src/sensing/hyper_lidar`) 패키지가 실차에 연결된 RPLidar 등 2D LiDAR 드라이버(`rplidar_ros`, rosdep으로 설치됨)를 실행합니다. 시뮬레이션에서는 사용하지 않으며, Gazebo가 `/scan`을 직접 발행합니다.

장치 포트, 보드레이트, 프레임 이름은 `config/rplidar_params.yaml`에서 실제 장비에 맞게 설정합니다 (기본값은 `/dev/rplidar` — 포트가 바뀌어도 흔들리지 않도록 GPS/카메라와 마찬가지로 idVendor/idProduct 기준 udev 심볼릭 링크를 걸어두는 것을 권장합니다).

```bash
ros2 launch hyper_lidar rplidar.launch.py
```

### USB 카메라

전방 ELP USB 카메라(ELP-USBGS1200P01-KL170, global shutter)는 `hyper_camera`의 `ElpCameraPublisherNode` 컴포넌트(실행 파일 `elp_camera_publisher_node`)가 `/dev/video_elp`를 열어 MJPEG을 캡처·디코드하고 자체적으로 rectify까지 처리한 뒤 `image_raw`로 발행합니다. `hyper_lane_detection`이 `input_backend:=intra_process`일 때 이 컴포넌트를 `lane_detection` 컴포넌트와 같은 `ComposableNodeContainer` 프로세스에 함께 로드해서, 프레임이 직렬화 없이 포인터로 바로 넘어갑니다(zero-copy intra-process) — 실차 전체/단계별 launch(`real.launch.py` → `sensors.launch.py` + `perception.launch.py`)는 기본적으로 이 경로를 씁니다.

`hyper_camera`(`src/sensing/hyper_camera`) 패키지는 이 노드와 그 보정 파일(`config/ELP-USBGS1200P01-KL170.yaml`)의 배포처입니다. `usb_cam`은 이 저장소에서 완전히 제거되었습니다.

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

규칙 적용 후 `hyper_camera`의 `config/params_elp.yaml`과 `ElpCameraPublisherNode`(파라미터 `video_device`) 둘 다 기본값으로 `/dev/video_elp`를 사용하므로, USB 포트가 바뀌어도 흔들리지 않습니다.

### Logitech C920 (객체 인식용)

객체 인식(YOLO)용 카메라로, `hyper_camera`의 `logitech_camera_publisher_node`가 `/dev/video_logitech`를 직접 열어(MJPEG 캡처) rectify 없이 원본 프레임을 그대로 `image_raw`로 발행합니다 — 별도 캘리브레이션 파일이 없습니다. ELP와 마찬가지로 `usb_cam`을 거치지 않지만, rclpy에는 rclcpp의 intra-process 통신에 해당하는 zero-copy 경로가 없으므로 이 쪽은 일반 ROS 토픽으로 `object_detection_node`에 연결됩니다 (`object_input_backend:=usb_camera`). 실차 전체 launch(`real.launch.py`)는 기본적으로 이 경로를 씁니다; `perception.launch.py` 자체의 기본값은 `ros_raw`(ros_gz_bridge 구독, 시뮬레이션/롤백용)입니다.

#### 1. 장치 udev 규칙

ELP/GPS와 동일하게 idVendor/idProduct 기준 udev 심볼릭 링크로 고정합니다. `/etc/udev/rules.d/99-logitech-camera.rules` 생성:

```
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="08e5", ATTR{index}=="0", SYMLINK+="video_logitech"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

다른 Logitech 모델을 쓴다면 idVendor/idProduct가 다를 수 있으니 `lsusb`와 `udevadm info -a -n /dev/videoN | grep -E "idVendor|idProduct"`로 직접 확인 후 값을 맞춰주세요. 규칙 적용 후 `object_detection_node`의 `video_device` 파라미터가 기본값으로 `/dev/video_logitech`를 사용하므로 USB 포트가 바뀌어도 흔들리지 않습니다. 해상도·프레임레이트는 `image_width`/`image_height`/`framerate` 파라미터로 조정합니다 (기본값은 `hyper_camera`의 `config/params_logitech.yaml`과 동일).

## 차량 제어 보드 설치 (실차)

`hyper_interface`의 `arduino_interface_node`가 `/velocity`[m/s], `/steering_angle`[rad] 명령 토픽을 그대로 구독해서 시리얼로 Arduino 보드에 전달하고, Arduino(`hyper_motor_interface.ino`)가 BTS7960(구동)/L298N(조향) 모터드라이버를 닫힌 루프로 구동합니다. `ros2_control` 경유 없이 명령 토픽 → Arduino로 직접 연결되는 구조입니다. 상세 배선/프로토콜/파라미터는 `src/interface/hyper_interface/README.md` 참고.

### 1. 장치 확인 및 의존성

```bash
sudo usermod -aG dialout $USER   # 적용하려면 재로그인 필요
sudo apt install python3-serial  # 또는 pip install pyserial
```

포트는 udev 규칙이 만드는 `/dev/tty_arduino`입니다([USB 시리얼 포트 고정](#usb-시리얼-포트-고정-udev--먼저-하세요) 참고). 이 보드는 CH340 칩이라 `/dev/ttyUSB*`로 잡히고, IMU 어댑터도 같은 자리를 노리므로 번호로 잡으면 안 됩니다:

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

`sim` 대신 `sensors`(EBIMU-9DOFV5 IMU + RPLidar + RTK)가 먼저 뜨고, 이후 `odometry`/`perception`/`behavior`는 시뮬레이션과 동일하게 staggered로 이어집니다. 카메라 둘(전방 ELP, 객체 인식용 Logitech C920)은 `sensors`가 아니라 `perception` 단계에서 열리므로 `sensors.launch.py`에는 포함되지 않습니다.

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
| E2BOX EBIMU-9DOFV5 | `hyper_ebimu` | `/imu` (EKF) |
| RPLidar | `hyper_lidar` | `/scan` |
| u-blox ZED-F9P x2 (dual RTK) + NTRIP | `hyper_rtk` | `/gps/fix`, `/imu/heading` |

카메라는 둘 다 여기 포함되지 않습니다 — `perception.launch.py`가 `lane_detection_container` 하나에 전방 ELP와 `lane_detection`을 함께 로드하고, 객체 인식용 Logitech C920만 별도 프로세스로 띄웁니다:

| 카메라 | 노드 | 토픽 | 전달 방식 |
|--------|------|------|-----------|
| 전방 ELP | `hyper_camera`의 `ElpCameraPublisherNode` (자체 rectify) | `/camera/image_raw` | intra-process (zero-copy) |
| 객체 인식 Logitech C920 | `hyper_camera`의 `logitech_camera_publisher_node` (rectify 없음) | `/camera_object/image_raw` | 일반 토픽 (rclpy에는 zero-copy 경로 없음) |

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
| `hyper_camera` | 실차 ELP/Logitech USB 카메라 드라이버 노드(`ElpCameraPublisherNode` C++ 컴포넌트, `logitech_camera_publisher_node` rclpy 노드) + 설정 파일(보정 파일 + 참고용 파라미터 yaml) 배포처 — `usb_cam`은 이 저장소에서 완전히 제거됨 |
| `hyper_interface` | 실차 ROS 2 ↔ Arduino 시리얼 브릿지 (`arduino_interface_node`) — `/velocity`, `/steering_angle`을 구독해 Arduino(`hyper_motor_interface.ino`)로 전달, BTS7960/L298N 모터드라이버를 닫힌 루프로 구동 |
---

## 주요 토픽

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo → ROS (시뮬레이션) / hyper_camera → hyper_lane_detection (실차) | 전방 카메라 영상 (차선 인식이 구독). 시뮬레이션은 ros_gz_bridge, 실차는 `hyper_camera`의 `ElpCameraPublisherNode`가 발행 — 실차 기본 경로(`lane_input_backend:=intra_process`)에서는 같은 `ComposableNodeContainer` 안에서 zero-copy로 전달되므로 이 토픽이 DDS까지 나가지 않음 |
| `/camera_object/image_raw` | `sensor_msgs/Image` | Gazebo → ROS (시뮬레이션) / hyper_camera → object_detection_node (실차) | 객체 인식용 카메라 영상 (`object_detection_node`가 항상 구독). 시뮬레이션은 ros_gz_bridge, 실차 기본 경로(`object_input_backend:=usb_camera`)는 `hyper_camera`의 `logitech_camera_publisher_node`가 일반 토픽으로 발행 |
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
| `/imu` | `sensor_msgs/Imu` | Gazebo → ROS (시뮬레이션) / `hyper_ebimu` → ROS (실차, EBIMU-9DOFV5) | IMU. 두 EKF가 roll/pitch·각속도·선가속도를 먹는다. yaw(지자기 융합)는 **안 쓴다**(`imu0_config` 인덱스 5가 양쪽 다 false) |
| `/imu/heading` | `sensor_msgs/Imu` | `rtk_heading` → `ekf_global` (`imu1`) | dual RTK 안테나 기선 기반 yaw. 스택의 **유일한 절대 방위 공급원**. 정지 상태에서도 나오지만 **RTK FIXED일 때만** 나온다 — 안 나오면 `rtk_heading`의 기각 사유 로그를 볼 것 |
