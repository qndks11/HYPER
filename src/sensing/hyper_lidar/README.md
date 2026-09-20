# hyper_lidar

실차에 연결된 RPLidar 등의 2D LiDAR 드라이버를 실행하기 위한 설정·launch 패키지입니다. 시뮬레이션에서는 사용하지 않으며, Gazebo가 `/scan`을 직접 발행합니다.

## 실행

```bash
ros2 launch hyper_lidar rplidar.launch.py
```

장치 포트, 보드레이트, 프레임 이름은 `config/rplidar_params.yaml`에서 실제 장비에 맞게 설정합니다.

드라이버는 360도 전체를 `/scan_raw`로 내보내고, 같은 launch의 `laser_filters` 노드(`scan_filter`)가 전방 180도(-90°..+90°)만 잘라 `/scan`으로 다시 발행합니다. 시뮬레이터 라이다(`vehicle.xacro`의 `gpu_lidar`)도 전방 180도라서, costmap은 실차와 시뮬에서 같은 모양의 `/scan`을 받습니다. 각도 범위는 `config/scan_filter.yaml`에서 바꿉니다. 필요한 패키지는 `ros-humble-laser-filters`이며, 루트 README의 `rosdep install`로 설치됩니다.

라이다는 거꾸로(윗면이 바닥을 향하게) 달려 있습니다. 이 보정은 드라이버(`inverted`)가 아니라 TF에서 합니다 — `vehicle.xacro`의 `lidar_joint`가 roll = π입니다. `inverted`까지 켜면 두 번 뒤집혀서 좌우가 다시 반대가 됩니다.

포트 기본값은 `/dev/rplidar`이고, 루트 `README.md`의 [USB 시리얼 포트 고정](../../../README.md#usb-시리얼-포트-고정-udev)에서 깔리는 `udev/99-hyper-serial.rules`가 이 이름을 붙입니다. 라이다 보드의 CP2102는 EBIMU USB-UART 어댑터와 VID:PID(`10c4:ea60`)가 같고 공장 기본 USB 시리얼도 양쪽 다 `0001`이라 원래는 구분이 안 됐습니다. 그래서 각 EEPROM에 이름을 써 넣었고(라이다 = `HYPER-LIDAR`), 규칙이 그 값으로 가릅니다 — `/dev/ttyUSB*` 번호로 잡으면 부팅마다 IMU와 자리가 바뀝니다. 다시 쓰는 방법은 [`udev/cp210x-serial.md`](../../../udev/cp210x-serial.md) 참고.

이 차의 라이다는 model 24 / fw 1.29 / hw 7 이고, USB 시리얼은 `HYPER-LIDAR`입니다. 연결만 확인하려면:

```bash
ls -l /dev/rplidar
ros2 launch hyper_lidar rplidar.launch.py
ros2 topic hz /scan_raw   # 드라이버 원본 (360도)
ros2 topic hz /scan       # 전방 180도
```
