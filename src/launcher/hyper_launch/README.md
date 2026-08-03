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

각 launch 파일은 각각 `hyper_gazebo`, `hyper_localization`, 인지 패키지, `hyper_planner`를 호출합니다. `sensors.launch.py`는 `realsense2_camera`(D435i, 객체 인식용 `/camera_object/image_raw` + EKF용 `/imu`), `hyper_lidar`(RPLidar, `/scan`), `hyper_rtk`(u-blox + NTRIP, `/gps/fix`)를 각 토픽 역할에 맞춰 묶어서 띄웁니다. 전방 ELP usb_cam은 더 이상 여기서 별도로 띄우지 않습니다 -- `perception.launch.py`가 `hyper_lane_detection`을 `input_backend:=direct_usb`로 실행하면서 `/dev/video_elp`를 직접 열고 자체적으로 보정(rectify)하기 때문입니다(`hyper_camera` 패키지 자체는 그대로 남아 있고, `ros2 launch hyper_camera camera.launch.py`로 독립 실행하거나 롤백용 `input_backend:=ros_compressed`와 함께 쓸 수 있습니다). 후방 카메라(`/camera_rear/image_raw`)는 아직 실물이 없어 발행되지 않고, `direct_usb`에는 후방 경로 자체가 없습니다.

`perception.launch.py`는 `lane_input_backend` 인자로 `hyper_lane_detection`의 `input_backend` 파라미터를 전달합니다. `real.launch.py`는 `direct_usb`를, `simulation.launch.py`는 `ros_raw`를 기본으로 넘깁니다.
