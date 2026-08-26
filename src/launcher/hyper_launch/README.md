# hyper_launch

HYPER 자율주행 스택을 단계별 또는 한 번에 실행하기 위한 launch 패키지입니다. 시뮬레이션(또는 실차 센서), 위치 추정, 인지, 행동 계획을 정해진 순서로 시작합니다.

## 실행

시뮬레이션 전체 스택:

```bash
ros2 launch hyper_launch simulation.launch.py
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
`remove_model_service` 노드도 함께 띄웁니다. `world_name`/`model_name` 파라미터(기본값
`course_world`/`accel_pedestrian`)로 지정된 모델을 gz 월드에서 제거하는 `std_srvs/srv/Trigger`
서비스입니다(예: `track.world`에 배치된 보행자 모델 제거).

```bash
ros2 service call /remove_model_service/remove std_srvs/srv/Trigger
```

매번 서비스 이름과 요청 필드를 외워서 커맨드라인으로 호출하기보다, GUI로 눌러서 부르고 싶다면
`rqt_service_caller`를 쓰면 됩니다 -- 실행 중인 모든 ROS 2 서비스가 드롭다운에 나오고, 위 서비스들은
전부 `Trigger`라 요청 필드 없이 Call 버튼만 누르면 됩니다.

```bash
ros2 run rqt_service_caller rqt_service_caller
```

실차 전체 스택:

```bash
ros2 launch hyper_launch real.launch.py
```

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

각 launch 파일은 각각 `hyper_gazebo`, `hyper_localization`, 인지 패키지, `hyper_planner`를 호출합니다. `sensors.launch.py`는 `witmotion_ros2`(WT901BLE, EKF용 `/imu`), `hyper_lidar`(RPLidar, `/scan`), `hyper_rtk`(u-blox + NTRIP, `/gps/fix`)를 각 토픽 역할에 맞춰 묶어서 띄웁니다.

카메라는 여기서 하나도 띄우지 않습니다 -- 전부 `perception.launch.py`가 엽니다. `lane_input_backend:=intra_process`는 `ComposableNodeContainer` 하나에 세 컴포넌트를 함께 로드합니다: `hyper_camera`의 `ElpCameraPublisherNode`(`/dev/video_elp` → `/camera/image_raw`), `realsense2_camera::RealSenseNodeFactory`(후방 D435i 컬러 → `/camera_rear/image_raw`, 설정은 `hyper_camera/config/params_d435i.yaml`), 그리고 `lane_detection`. 셋 다 `use_intra_process_comms`가 켜져 있어 두 카메라 모두 프레임이 직렬화 없이 포인터로 `lane_detection`에 전달됩니다. `object_input_backend:=usb_camera`는 `logitech_camera_publisher_node`를 별도 프로세스로 띄워 `/dev/video_logitech`를 열고 일반 토픽으로 `object_detection_node`에 넘깁니다(rclpy에는 zero-copy 경로가 없음).

`usb_cam`은 이 저장소에서 완전히 제거되었고(`deps.repos`에서도 삭제됨), `hyper_camera` 패키지가 이제 두 USB 카메라의 드라이버 노드와 D435i를 포함한 모든 카메라 설정 yaml의 배포처입니다. 후방 카메라는 실차에서 컬러만 발행하며, 깊이·포인트 클라우드(`/camera_rear/depth/image_raw`, `/camera_rear/depth/points`)는 소비자가 없어 꺼져 있어 시뮬레이션에서만 나옵니다.

`perception.launch.py`는 `lane_input_backend`/`object_input_backend` 인자로 각 인지 노드의 카메라 소스를 고릅니다. `real.launch.py`는 `intra_process`/`usb_camera`를, `simulation.launch.py`는 `ros_raw`(둘 다)를 기본으로 넘깁니다.
