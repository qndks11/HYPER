# hyper_launch

HYPER 자율주행 스택을 단계별 또는 한 번에 실행하기 위한 launch 패키지입니다. 시뮬레이션(또는 실차 센서), 위치 추정, 인지, 행동 계획을 정해진 순서로 시작합니다.

## 실차 Common:

단계별 실행 (실차):

Terminal 1: Arduino
```bash
ros2 launch hyper_launch interface.launch.py 
```

Terminal 2: TF tree
```bash
ros2 launch hyper_control robot_state_publisher.launch.py
```

Terminal 3: Sensors
```bash
ros2 launch hyper_launch sensors.launch.py
```

Terminal 4: Localization
```bash
ros2 launch hyper_launch odometry.launch.py datum_site:=school use_sim_time:=false
```

Terminal 5: GPS Monitor (optional)
```bash
ros2 run hyper_localization gps_accuracy_gui.py      # 절대 위치(base): hAcc/vAcc, fix/RTK, x/y
```

```bash
ros2 run hyper_waypoint_studio waypoint_studio
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
  mission:=mission_school
```

