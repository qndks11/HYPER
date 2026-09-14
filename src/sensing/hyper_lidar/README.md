# hyper_lidar

실차에 연결된 RPLidar 등의 2D LiDAR 드라이버를 실행하기 위한 설정·launch 패키지입니다. 시뮬레이션에서는 사용하지 않으며, Gazebo가 `/scan`을 직접 발행합니다.

## 실행

```bash
ros2 launch hyper_lidar rplidar.launch.py
```

장치 포트, 보드레이트, 프레임 이름은 `config/rplidar_params.yaml`에서 실제 장비에 맞게 설정합니다.

포트 기본값은 `/dev/rplidar`이고, 루트 `README.md`의 [USB 시리얼 포트 고정](../../../README.md#usb-시리얼-포트-고정-udev)에서 깔리는 `udev/99-hyper-serial.rules`가 이 이름을 붙입니다. 라이다 보드의 CP2102는 EBIMU USB-UART 어댑터와 VID:PID(`10c4:ea60`)가 같고 공장 기본 USB 시리얼도 양쪽 다 `0001`이라 원래는 구분이 안 됐습니다. 그래서 각 EEPROM에 이름을 써 넣었고(라이다 = `HYPER-LIDAR`), 규칙이 그 값으로 가릅니다 — `/dev/ttyUSB*` 번호로 잡으면 부팅마다 IMU와 자리가 바뀝니다. 다시 쓰는 방법은 [`udev/cp210x-serial.md`](../../../udev/cp210x-serial.md) 참고.

이 차의 라이다는 model 24 / fw 1.29 / hw 7 이고, USB 시리얼은 `HYPER-LIDAR`입니다. 연결만 확인하려면:

```bash
ls -l /dev/rplidar
ros2 launch hyper_lidar rplidar.launch.py
ros2 topic hz /scan
```
