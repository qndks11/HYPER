# hyper_launch

HYPER 자율주행 스택을 단계별 또는 한 번에 실행하기 위한 launch 패키지입니다. 시뮬레이션(또는 실차 센서), 위치 추정, 인지, 행동 계획을 정해진 순서로 시작합니다.

## 실행

시뮬레이션 전체 스택:

```bash
ros2 launch hyper_launch simulation.launch.py
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

각 launch 파일은 각각 `hyper_gazebo`, `hyper_localization`, 인지 패키지, `hyper_planner`를 호출합니다. `sensors.launch.py`는 `realsense2_camera`(D435i, EKF용 `/imu`만 사용 — `enable_color`/`enable_depth`는 꺼져 있음), `hyper_lidar`(RPLidar, `/scan`), `hyper_rtk`(u-blox + NTRIP, `/gps/fix`)를 각 토픽 역할에 맞춰 묶어서 띄웁니다. 전방 ELP와 객체 인식용 Logitech C920은 여기서 띄우지 않습니다 -- `perception.launch.py`가 `hyper_camera`의 드라이버 노드로 각각 엽니다: `lane_input_backend:=intra_process`는 `ElpCameraPublisherNode` 컴포넌트를 `lane_detection_node`와 같은 `ComposableNodeContainer`에 함께 로드해 `/dev/video_elp`를 열고(zero-copy intra-process 전달), `object_input_backend:=usb_camera`는 `logitech_camera_publisher_node`를 별도 프로세스로 띄워 `/dev/video_logitech`를 열고 일반 토픽으로 `object_detection_node`에 넘깁니다. `usb_cam`은 이 저장소에서 완전히 제거되었고(`deps.repos`에서도 삭제됨), `hyper_camera` 패키지가 이제 두 카메라의 드라이버 노드와 설정 yaml 배포처입니다. 후방 카메라(`/camera_rear/image_raw`)는 아직 실물이 없어 발행되지 않고, `intra_process`에는 후방 경로 자체가 없습니다.

`perception.launch.py`는 `lane_input_backend`/`object_input_backend` 인자로 각 인지 노드의 카메라 소스를 고릅니다. `real.launch.py`는 `intra_process`/`usb_camera`를, `simulation.launch.py`는 `ros_raw`(둘 다)를 기본으로 넘깁니다.
