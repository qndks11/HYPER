# hyper_launch

HYPER 자율주행 스택을 단계별 또는 한 번에 실행하기 위한 launch 패키지입니다. 시뮬레이션(또는 실차 센서), 위치 추정, 인지, 행동 계획을 정해진 순서로 시작합니다.

## 시뮬레이션 전체 스택:

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

## 실차 Common:

단계별 실행 (실차):

Terminal 1: TF tree
```bash
ros2 launch hyper_control robot_state_publisher.launch.py
```

Terminal 2: Sensors
```bash
ros2 launch hyper_launch sensors.launch.py
```

EBIMU(`/imu`), RPLidar(`/scan`)와 함께 `hyper_rtk`(u-blox ZED-F9P 2대)를 띄웁니다. base는
NTRIP 보정으로 절대 위치를 `/gps/fix`에, rover는 moving-base RTK 헤딩을 `/imu/heading`에
냅니다.

Terminal 3: Localization
```bash
ros2 launch hyper_launch odometry.launch.py datum_site:=school use_sim_time:=false
```

Terminal 4: Arduino
```bash
ros2 launch hyper_launch interface.launch.py 
```

Terminal 5: RTK Monitor (optional)
```bash
# 절대 위치(base: hAcc/vAcc, fix/RTK, x/y)와 절대 방위(rover: RTK FIXED /
# heading valid, 나침반)를 한 창에서 본다. 나침반 바늘은 IMU가 아니라 듀얼
# 안테나 baseline에서 온다 -- 채운 삼각형이 RTK 원본, 가는 선이 EKF yaw.
ros2 run hyper_localization gps_accuracy_gui.py
```

Terminal 6: Mission panel (optional)
```bash
ros2 run hyper_rqt hyper_panel 
```

### Real Car Joystick & Waypoint record
Terminal 7: Emergency stop

```bash
ros2 launch hyper_control estop.launch.py estop_button_index:=1 resume_button_index:=0
```

Terminal 8: Joystick
```bash
ros2 launch hyper_control joystick.launch.py joystick_publish_period:=0.0
```

Terminal 9: Waypoint Recorder (Optional)
```bash
ros2 launch hyper_waypoint record.launch.py \
  waypoint_csv:=$HOME/HYPER/src/planning/hyper_waypoint/waypoints/real.csv \
  min_spacing_m:=0.5 use_record_gui:=true
```

### Real car Mission
Terminal 7: Emergency Stop (optional)
```bash
ros2 launch hyper_control estop.launch.py \
  estop_button_index:=1 resume_button_index:=0 launch_joy_node:=true
```

Terminal 8: Perception
```bash
ros2 launch hyper_launch perception.launch.py \
  lane_input_backend:=intra_process
```

Terminal 9: Mission
```bash
ros2 launch hyper_launch behavior.launch.py \
  use_sim_time:=false \
  mission:=mission \
  waypoint_csv:=$HOME/HYPER/src/planning/hyper_waypoint/waypoints/real.csv
```