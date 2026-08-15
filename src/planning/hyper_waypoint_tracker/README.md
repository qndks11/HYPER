# hyper_waypoint_tracker

Standalone absolute-GPS-waypoint recording and pure-pursuit tracking.
Independent of `hyper_planner`'s event/`course.yaml` system: this package
follows a plain ordered list of absolute lat/lon waypoints, regardless of
where the vehicle starts.

## Why lat/lon, not map-frame x/y

This stack's `map`/`odom` frame is not stable across runs:
`dual_ekf_navsat.yaml` sets no fixed `datum` and `wait_for_datum: false`, so
`navsat_transform_node` re-anchors the frame's origin to wherever the first
`/gps/fix` lands *each session*. Waypoints stored as `/odometry/filtered_map`
x/y would only replay correctly if every run started from the exact same
spot/heading. Recording raw lat/lon instead gives truly absolute waypoints;
`waypoint_tracker_node` converts them into the current session's map frame
at startup (see Calibration below).

## 1. Record waypoints

```bash
source install/setup.bash
python3 src/planning/hyper_waypoint_tracker/scripts/waypoint_recorder.py
```

Drive to the start line, then:

```bash
ros2 topic pub --once /waypoint_tracker/cmd std_msgs/msg/String "{data: 'start'}"
# ... drive the course ...
ros2 topic pub --once /waypoint_tracker/cmd std_msgs/msg/String "{data: 'stop'}"
ros2 topic pub --once /waypoint_tracker/cmd std_msgs/msg/String "{data: 'save'}"
```

A clean `Ctrl-C` also auto-saves. Output goes to
`waypoints/waypoints.yaml` (`{waypoints: [{latitude, longitude}, ...]}`) by
default; override with the `output_path` parameter.

## 2. Track waypoints

```bash
ros2 launch hyper_waypoint_tracker waypoint_tracker.launch.py
```

`waypoint_tracker_node` publishes `/velocity` and `/steering_angle`
(`std_msgs/Float64`) -- the same actuation interface `vehicle_controller_node`
(in `hyper_control`) already consumes, so this drives the vehicle standalone.

### Calibration

On startup the node buffers live `(/gps/fix, /odometry/filtered_map)` sample
pairs for `calibration_window_s` (default 2s) and averages the translation
offset between the waypoints' GPS-relative local frame and the live map
frame, then applies it to every waypoint. This assumes `map`/`odom` axes are
ENU-aligned (x=East, y=North), which is `navsat_transform_node`'s standard
behavior given this stack's `magnetic_declination_radians`/`yaw_offset`
config. No calibration drive is needed -- the vehicle can be stationary
during this window, as long as GPS + the localization stack are already
running and locked.

### Halt

The node holds `velocity = 0`, `steering_angle = 0` once it's within
`goal_radius_m` of the final waypoint (or the pursuit target falls behind
the vehicle at the end of the path) -- it does not loop or idle-drift.

## Params

See `config/waypoint_tracker_params.yaml`: `wheelbase`, `control_hz`,
`lookahead`, `cruise_speed`, `max_steering_angle`,
`max_steering_angular_velocity`, `max_velocity`, `accel_limit`,
`decel_limit`, `goal_radius_m`, `calibration_window_s`, `input_timeout`.
`waypoints_yaml` is set via the launch argument, not this file.
