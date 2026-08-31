# hyper_launch

HYPER 자율주행 스택을 단계별 또는 한 번에 실행하기 위한 launch 패키지입니다. 시뮬레이션(또는 실차 센서), 위치 추정, 인지, 행동 계획을 정해진 순서로 시작합니다.

## 실행

시뮬레이션 전체 스택:

```bash
ros2 launch hyper_launch simulation.launch.py datum_site:=school headless:=true drivable_area:=true
```

`behavior.launch.py`가 띄우는 `mission_manager_node`는 `auto_start` 기본값이 `false`라 노드가 뜬
뒤에도 대기만 합니다. 실제로 미션 주행을 시작하려면 서비스 콜이 한 번 더 필요합니다(미션 주행은
사람이 시작하는 게 안전하므로 의도된 동작입니다). `mission_manager_node`는 전부 `std_srvs/srv/Trigger`인
서비스 4개를 노출합니다.

```bash
ros2 service call /mission_manager/start std_srvs/srv/Trigger    # 미션 시작
ros2 service call /mission_manager/cancel std_srvs/srv/Trigger   # 현재 미션 취소
ros2 service call /mission_manager/skip std_srvs/srv/Trigger     # 현재 스텝 건너뛰기
ros2 service call /mission_manager/restart std_srvs/srv/Trigger  # 처음부터 재시작
```

시뮬레이션에서는 `vehicle.launch.py`(`sim.launch.py`/`simulation.launch.py`가 포함)가
`model_service` 노드도 함께 띄웁니다. `world_name`/`model_name`/`model_uri`/`pose` 파라미터로
지정된 모델을 gz 월드에 띄우거나 지우는 `std_srvs/srv/Trigger` 서비스 두 개입니다(기본값은
`track.world`의 가속 구간 보행자).

```bash
ros2 service call /model_service/spawn std_srvs/srv/Trigger
ros2 service call /model_service/remove std_srvs/srv/Trigger
```

매번 서비스 이름을 외워서 커맨드라인으로 호출하기보다 GUI로 눌러서 부르고 싶다면
`hyper_rqt` 패널을 쓰세요. 위 서비스들만 버튼으로 모아 두고 `/mission_manager/status`를 맨 위에
보여 줍니다.

```bash
ros2 run hyper_rqt hyper_panel
```

`rqt_service_caller`도 물론 되지만 드롭다운에 실행 중인 서비스가 전부 나옵니다 -- 노드마다
파라미터 서비스가 6개씩 붙어서 `simulation.launch.py` 기준 100개가 넘습니다. 버튼을 추가하거나
고치려면 `src/tools/hyper_rqt/config/panel.yaml`을 보세요(코드가 아니라 YAML입니다).

실차 전체 스택 (미션 주행):

```bash
ros2 launch hyper_launch real.launch.py datum_site:=track mission:=simple \
  waypoint_csv:=$HOME/HYPER/src/planning/hyper_waypoint/waypoints/real.csv
```

`waypoint_csv`의 기본값은 `sim.csv`입니다. 실차에서는 녹화한 코스로 반드시 덮어쓰세요.

실차 수동 주행 (조이스틱, 웨이포인트 녹화용):

```bash
ros2 launch hyper_launch joystick_control_real.launch.py datum_site:=track
```

`joystick_control_real.launch.py`와 `phone_control_real.launch.py`는 둘 다 웨이포인트
레코더와 녹화 조작판 GUI를 포함합니다(`hyper_waypoint/record.launch.py`). 레코더는
`auto_start:=false`로 떠서 GUI의 `● Record`를 누를 때까지 기다립니다 -- 기록 시작이 곧
CSV truncate이므로, 스택을 띄우는 것만으로 지난 녹화본이 날아가지 않습니다. 녹화 대상
파일과 간격은 `waypoint_csv:=`, `min_spacing_m:=`으로 바꾸고, GUI가 필요 없으면
`use_record_gui:=false`입니다. 자세한 내용은
[hyper_waypoint/README.md](../../planning/hyper_waypoint/README.md)를 보세요.

조이스틱 쪽은 카메라와 차선/객체 인식을 아예 띄우지 않습니다. 수동 주행과 녹화는
`/odometry/filtered_map`만 쓰므로 인지 스택이 하는 일이 없고, USB 카메라 두 대와 추론
노드는 놀고 있지 않습니다. 실차에서 인지를 보려면 `perception.launch.py`를 따로 띄우세요.

수동 주행과 미션은 launch가 나뉘어 있고 **같이 띄우면 안 됩니다**.
`joystick_controller_node`는 스틱을 안 건드려도 `/velocity` + `/steering_angle`을 100Hz로 계속
내보내는데, 이는 behavior 스테이지의 `cmd_vel_to_ackermann_node`가 nav2의 `/cmd_vel`을 변환해
내보내는 토픽과 똑같습니다. 둘을 같이 띄우면 스틱 중립(0.0) 명령이 nav2 명령과 번갈아
`arduino_interface_node`에 도착해 차가 자율주행을 못 합니다. (예전 `real.launch.py`의
`use_joystick` 인자가 바로 그 사고를 한 글자로 낼 수 있는 스위치라 제거했습니다.)

하루치 실차 테스트 절차는 [AUG30.md](AUG30.md)에 있습니다.

단계별 실행 (시뮬레이션):

```bash
ros2 launch hyper_launch sim.launch.py
ros2 launch hyper_launch odometry.launch.py
ros2 launch hyper_launch perception.launch.py
ros2 launch hyper_launch behavior.launch.py
```

단계별 실행 (실차):

```bash
ros2 launch hyper_launch sensors.launch.py
ros2 launch hyper_launch odometry.launch.py
ros2 launch hyper_launch perception.launch.py
ros2 launch hyper_launch behavior.launch.py
```

각 launch 파일은 각각 `hyper_gazebo`, `hyper_localization`, 인지 패키지, `hyper_planner`를 호출합니다. `sensors.launch.py`는 `hyper_ebimu`(EBIMU-9DOFV5, EKF용 `/imu`), `hyper_lidar`(RPLidar, `/scan`), `hyper_rtk`(u-blox + NTRIP, `/gps/fix`)를 각 토픽 역할에 맞춰 묶어서 띄웁니다.

카메라는 여기서 하나도 띄우지 않습니다 -- 전부 `perception.launch.py`가 엽니다. `lane_input_backend:=intra_process`는 `ComposableNodeContainer` 하나에 두 컴포넌트를 함께 로드합니다: `hyper_camera`의 `ElpCameraPublisherNode`(`/dev/video_elp` → `/camera/image_raw`)와 `lane_detection`. 둘 다 `use_intra_process_comms`가 켜져 있어 프레임이 직렬화 없이 포인터로 `lane_detection`에 전달됩니다. `object_input_backend:=usb_camera`는 `logitech_camera_publisher_node`를 별도 프로세스로 띄워 `/dev/video_logitech`를 열고 일반 토픽으로 `object_detection_node`에 넘깁니다(rclpy에는 zero-copy 경로가 없음).

`usb_cam`은 이 저장소에서 완전히 제거되었고(`deps.repos`에서도 삭제됨), `hyper_camera` 패키지가 이제 두 USB 카메라의 드라이버 노드와 카메라 설정 yaml의 배포처입니다. 후방 RealSense D435i는 배터리 절약을 위해 실차·시뮬레이션 양쪽에서 완전히 제거되었습니다 -- `/camera_rear/*`와 `/scan_rear`는 더 이상 존재하지 않습니다. 두 카메라 모두 640x360@30으로 엽니다(`hyper_camera` README 참고).

`perception.launch.py`는 `lane_input_backend`/`object_input_backend` 인자로 각 인지 노드의 카메라 소스를 고릅니다. `real.launch.py`는 `intra_process`/`usb_camera`를, `simulation.launch.py`는 `ros_raw`(둘 다)를 기본으로 넘깁니다.
