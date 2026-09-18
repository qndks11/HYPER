# hyper_lidar

실차에 연결된 RPLidar 등의 2D LiDAR 드라이버를 실행하기 위한 설정·launch 패키지입니다. 시뮬레이션에서는 사용하지 않으며, Gazebo가 `/scan`을 직접 발행합니다.

## 실행

```bash
ros2 launch hyper_lidar rplidar.launch.py
```

이 launch는 노드를 **둘** 띄웁니다:

| 노드 | 토픽 | 하는 일 |
| --- | --- | --- |
| `rplidar_node` | → `/scan_raw` | 드라이버. 360도 원본 |
| `scan_front_filter` | `/scan_raw` → `/scan` | 전방 180도 밖의 빔을 NaN으로 지움 |

스택의 나머지(nav2 `local_costmap` 포함)는 예전 그대로 `/scan`만 봅니다.

## 왜 전방 180도로 자르는가 — 뒤쪽 코스트맵 유지

라이다는 앞 범퍼 위에 있지만 실제로는 360도를 다 훑어서 뒤쪽 빔까지 `/scan`에 싣습니다.
그대로 nav2에 넣으면 **차가 지나온 장애물이 뒤로 빠지는 즉시 코스트맵에서 지워집니다.**
`nav2_controller.yaml`의 `obstacle_layer`는 스캔을 `marking`뿐 아니라 `clearing`에도 쓰기
때문입니다. 지우는 경로는 둘입니다:

1. 뒤가 비어 있으면 `rplidar_ros`가 그 빔을 `inf`로 냅니다. scan 소스는 `inf_is_valid: true`라
   `inf`를 `range_max - eps`로 바꿔 정상 빔처럼 레이캐스팅하므로, `raytrace_max_range`(20 m)
   까지 전부 FREE가 됩니다.
2. `obstacle_max_range`(9 m)보다 멀리 뒤로 빠진 장애물은 마킹 대상이 아닌데
   `raytrace_max_range`는 20 m라, 다시 마킹되지 못한 채 지워지기만 합니다.

시뮬레이션 라이다는 애초에 전방 180도만 봅니다(`hyper_control/urdf/vehicle.xacro`의
`<horizontal>` `min/max_angle` ±π/2). 그건 성능 타협이 아니라 **의도된 설계**입니다 —
뒤를 보는 빔이 없으면 뒤를 지울 방법도 없으므로, 전진하며 마킹한 셀이 rolling window
(30×30 m)에서 밀려날 때까지 남습니다. `scan_front_filter`는 실차를 그 모델에 맞춥니다.

빔을 **자르지 않고 NaN으로 덮는** 이유는 `scripts/scan_front_filter.py` 헤더에 적혀
있습니다(요약: 드라이버의 각도 규약과 무관하게 동작하고, `angle_min/max`가 그대로라
RViz 기하가 변하지 않으며, NaN은 `laser_geometry`가 통째로 버려 마킹도 레이캐스팅도
일으키지 않습니다 — `0.0`이나 `inf`로 덮으면 nav2가 `inf`를 유효 거리로 되살립니다).

섹터를 바꾸려면 `scan_front_filter`의 `lower_angle` / `upper_angle`(rad, 기본 ∓π/2)을
쓰되 **`vehicle.xacro`의 FOV도 같이 바꾸세요.** 두 값이 어긋나면 실차와 시뮬레이션의
코스트맵이 다르게 차고 지워집니다.

360도 원본을 그대로 보고 싶으면 필터를 끕니다(그 상태로 미션을 돌리면 다시 뒤쪽이
지워집니다):

```bash
ros2 launch hyper_lidar rplidar.launch.py use_front_filter:=false
```

장치 포트, 보드레이트, 프레임 이름은 `config/rplidar_params.yaml`에서 실제 장비에 맞게 설정합니다.

포트 기본값은 `/dev/rplidar`이고, 루트 `README.md`의 [USB 시리얼 포트 고정](../../../README.md#usb-시리얼-포트-고정-udev)에서 깔리는 `udev/99-hyper-serial.rules`가 이 이름을 붙입니다. 라이다 보드의 CP2102는 EBIMU USB-UART 어댑터와 VID:PID(`10c4:ea60`)가 같고 공장 기본 USB 시리얼도 양쪽 다 `0001`이라 원래는 구분이 안 됐습니다. 그래서 각 EEPROM에 이름을 써 넣었고(라이다 = `HYPER-LIDAR`), 규칙이 그 값으로 가릅니다 — `/dev/ttyUSB*` 번호로 잡으면 부팅마다 IMU와 자리가 바뀝니다. 다시 쓰는 방법은 [`udev/cp210x-serial.md`](../../../udev/cp210x-serial.md) 참고.

이 차의 라이다는 model 24 / fw 1.29 / hw 7 이고, USB 시리얼은 `HYPER-LIDAR`입니다. 연결만 확인하려면:

```bash
ls -l /dev/rplidar
ros2 launch hyper_lidar rplidar.launch.py
ros2 topic hz /scan_raw   # 드라이버 원본
ros2 topic hz /scan       # 필터 출력 (같은 Hz여야 정상)
```

무반사 빔을 드라이버가 뭘로 내는지도 여기서 확인하세요. `ros2 topic echo /scan_raw`에
`.inf`가 보이면 `nav2_controller.yaml`의 `inf_is_valid: true`가 맞게 동작하는 것이고,
`0.0`으로 나온다면 그 빔은 레이캐스팅에 아예 안 쓰입니다(그쪽 주석 참고).
