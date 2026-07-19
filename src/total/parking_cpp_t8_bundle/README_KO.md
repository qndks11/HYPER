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
rm -rf build/parking_cpp install/parking_cpp
colcon build --packages-select parking_cpp --symlink-install
source install/setup.bash
ros2 launch parking_cpp parking_system_cpp.launch.py
```
