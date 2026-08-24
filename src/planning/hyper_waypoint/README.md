# hyper_waypoint

`odometry/filtered_map`을 구독해서 `idx, x, y, yaw, frame_id`를 CSV로 기록하는 웨이포인트 레코더 패키지입니다.

## 실행

```bash
colcon build --packages-select hyper_waypoint
source install/setup.bash
ros2 run hyper_waypoint waypoint_recorder_node --ros-args -p output_csv:=$HOME/HYPER/src/planning/hyper_waypoint/waypoints/sim.csv -p min_spacing_m:=0.5
```

- `output_csv` 파라미터를 생략하면 노드를 실행한 위치에 `waypoint_record.csv`로 저장됩니다.
- `min_spacing_m` 파라미터(기본값 `0.5`)는 직전 기록 지점으로부터 이 거리(m) 이상 이동했을 때만 새 줄을 기록합니다. 시간 간격이 아니라 이동 거리 기준으로 웨이포인트가 샘플링됩니다.
- 노드가 실행 중인 동안 `odometry/filtered_map`으로 들어오는 메시지 중 `min_spacing_m` 이상 이동한 시점마다 한 줄씩 기록됩니다. 필요한 구간만 남기려면 `Ctrl-C`로 원하는 시점에 종료하세요.
- 파일은 `idx,x,y,yaw,frame_id` 헤더로 시작하며, 매 기록마다 flush되므로 중간에 종료해도 그때까지 기록된 내용은 남아 있습니다.
