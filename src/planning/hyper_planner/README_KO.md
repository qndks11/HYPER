# 교차로 + 경사로 5초 정지 이벤트 버전

주차 기능, Hybrid A*, 주차 costmap을 제거했다.

상태 흐름:
- 교차로: LANE_FOLLOW -> APPROACH -> STOP_AT_LIGHT/TURN_BRIDGE -> LANE_FOLLOW
- 경사로: LANE_FOLLOW -> HILL_APPROACH -> HILL_STOP(기본 5초) -> LANE_FOLLOW

경사로 이벤트 YAML 예:
```yaml
slope_A:
  event_type: hill_stop
  latitude: 37.0
  longitude: 127.0
  approach_radius_m: 2.5
  stop_duration_s: 5.0
  signal_required: false
  paths: {}
```

빌드:
```bash
cd ~/HYPER
rm -rf build/hyper_planner install/hyper_planner
colcon build --packages-select hyper_planner --symlink-install
source install/setup.bash
ros2 launch hyper_planner parking_system_cpp.launch.py
```

## 특정 가속구간 장애물 정지

GPS로 등록된 `accel_obstacle` 이벤트 안에서만 `/scan`을 감시한다.
라이다 좌표계의 +X를 차량 전방으로 보고, 전방 직사각형 영역의 장애물을 판단한다.

상태 흐름:
- `LANE_FOLLOW -> ACCEL_OBSTACLE_ZONE`
- 장애물 또는 LiDAR timeout: `ACCEL_OBSTACLE_ZONE -> OBSTACLE_STOP`
- 장애물이 설정 거리 밖에서 일정 시간 유지: `OBSTACLE_STOP -> ACCEL_OBSTACLE_ZONE`
- 설정된 구간 길이 주행 완료: `ACCEL_OBSTACLE_ZONE -> LANE_FOLLOW`

등록 명령 예:
```bash
# 가속 구간 시작 위치에서
ros2 topic pub --once /waypoint/cmd std_msgs/msg/String "{data: 'mark_accel_start:accel_A'}"

# 가속 구간 끝 위치로 이동한 뒤
ros2 topic pub --once /waypoint/cmd std_msgs/msg/String "{data: 'mark_accel_end:accel_A'}"

# 시작점/끝점 진입 반경
ros2 topic pub --once /waypoint/cmd std_msgs/msg/String "{data: 'set_accel_radius:accel_A:3.0:3.0'}"
ros2 topic pub --once /waypoint/cmd std_msgs/msg/String "{data: 'set_accel_speed:accel_A:4.0'}"
ros2 topic pub --once /waypoint/cmd std_msgs/msg/String "{data: 'set_obstacle_distance:accel_A:2.0:2.5'}"
ros2 topic pub --once /waypoint/cmd std_msgs/msg/String "{data: 'set_obstacle_width:accel_A:0.55'}"
ros2 topic pub --once /waypoint/cmd std_msgs/msg/String "{data: 'set_obstacle_clear_hold:accel_A:1.0'}"
ros2 topic pub --once /waypoint/cmd std_msgs/msg/String "{data: 'save'}"
```

주의:
- RViz의 TF 표시에서 `lidar_link` 빨간색 +X축이 차량 전방을 향해야 한다.
- 현재 제어 속도는 `parking_params.yaml`의 `controller.accel_zone_speed`를 사용한다.
- 이벤트 YAML의 `target_speed_mps`는 기록용이며, 이벤트별 속도를 다르게 쓰려면 속도 토픽을 추가해야 한다.
