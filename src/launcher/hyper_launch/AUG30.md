Waypoint check w/ visualization

```bash
python3 src/planning/hyper_waypoint/scripts/plot_waypoints.py \
  src/planning/hyper_waypoint/waypoints/sim.csv --jump-threshold 1.0
```

Waypoint labeling

```bash
python3 src/planning/hyper_waypoint/scripts/label_waypoints.py \
  src/planning/hyper_waypoint/waypoints/real.csv \
  --mission src/planning/hyper_planner/config/stopline.yaml
```