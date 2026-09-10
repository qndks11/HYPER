Waypoint check w/ visualization

```bash
# plot_waypoints.py는 없어졌습니다 (레코더가 진단 컬럼을 더 이상 안 적습니다).
# 코스를 보려면: ros2 run hyper_waypoint_studio waypoint_studio \
  src/planning/hyper_waypoint/waypoints/real.csv --jump-threshold 1.0
```

Waypoint labeling

```bash
ros2 run hyper_waypoint_studio waypoint_studio --mode edit \
  src/planning/hyper_waypoint/waypoints/real.csv \
  --mission src/planning/hyper_planner/mission/stopline.yaml
```