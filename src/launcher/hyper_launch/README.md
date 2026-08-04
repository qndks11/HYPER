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

각 launch 파일은 각각 `hyper_gazebo`, `hyper_localization`, 인지 패키지, `hyper_planner`를 호출합니다. `sensors.launch.py`는 `realsense2_camera`(D435i, EKF용 `/imu`만 사용 — `enable_color`/`enable_depth`는 꺼져 있음), `hyper_lidar`(RPLidar, `/scan`), `hyper_rtk`(u-blox + NTRIP, `/gps/fix`)를 각 토픽 역할에 맞춰 묶어서 띄웁니다. 전방 ELP와 객체 인식용 Logitech C920은 더 이상 여기서 띄우지 않습니다 -- `perception.launch.py`가 `hyper_lane_detection`을 `lane_input_backend:=direct_usb`로, `object_detection_node`를 `object_input_backend:=direct_usb`로 실행하면서 각각 `/dev/video_elp`/`/dev/video_logitech`를 직접 열기 때문입니다. `usb_cam`은 이 저장소에서 완전히 제거되었고(`deps.repos`에서도 삭제됨), `hyper_camera` 패키지는 이제 두 카메라의 설정 yaml 배포처로만 남아 있습니다. 후방 카메라(`/camera_rear/image_raw`)는 아직 실물이 없어 발행되지 않고, `direct_usb`에는 후방 경로 자체가 없습니다.

`perception.launch.py`는 `lane_input_backend` 인자로 `hyper_lane_detection`의 `input_backend` 파라미터를 전달합니다. `real.launch.py`는 `direct_usb`를, `simulation.launch.py`는 `ros_raw`를 기본으로 넘깁니다.
