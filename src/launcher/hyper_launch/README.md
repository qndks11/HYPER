# hyper_launch

HYPER 자율주행 스택을 단계별 또는 한 번에 실행하기 위한 launch 패키지입니다. 시뮬레이션(또는 실차 센서), 위치 추정, 인지, 행동 계획을 정해진 순서로 시작합니다.

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
ros2 launch hyper_launch odometry.launch.py datum_site:=track use_sim_time:=false
```

Terminal 4: Arduino
```bash
ros2 launch hyper_launch interface.launch.py 
```

Terminal 5: GPS Monitor (optional)
```bash
ros2 run hyper_localization gps_accuracy_gui.py      # 절대 위치(base): hAcc/vAcc, fix/RTK, x/y
```

Terminal 6: Mission panel (optional)
```bash
ros2 run hyper_rqt hyper_panel 
```

Terminal 7: Waypoint View & Recorder (Optional)
```bash
ros2 launch hyper_waypoint record.launch.py \
  waypoint_csv:=$HOME/HYPER/src/planning/hyper_waypoint/waypoints/track.csv \
  min_spacing_m:=0.5 use_record_gui:=true
```

### Real Car Joystick & Waypoint record
Terminal 7: Joystick
```bash
ros2 launch hyper_control joystick.launch.py joystick_publish_period:=0.0
```

### Real car Mission

Terminal 7: Perception (optional)
```bash
ros2 launch hyper_launch perception.launch.py \
  lane_input_backend:=intra_process
```

Terminal 8: Mission
```bash
ros2 launch hyper_launch behavior.launch.py \
  use_sim_time:=false \
  mission:=simple \
  waypoint_csv:=$HOME/HYPER/src/planning/hyper_waypoint/waypoints/real.csv
```