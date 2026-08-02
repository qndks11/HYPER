# CLAUDE.md

## Project

HYPER is a ROS 2 (Humble) autonomous-driving stack built for **HL FMA (Future Mobility Award) 2026**, the **[1/5] category** ("aMAP Innovator Championship") — a university-division competition run by HL Mando, HL Klemove, and Halla University, using 1/5-scale autonomous vehicles.

## Repo layout

Colcon workspace rooted here (`~/HYPER`). `build/`, `install/`, `log/` are gitignored build artifacts.

- `src/control/hyper_control` (ROS package `hyper_control`) — core package: Ackermann vehicle control, joystick teleop, lane/stopline detection, YOLO-based object/traffic-light detection. MIT license.
- `src/simulator/hyper_gazebo` (ROS package `hyper_gazebo`) — Gazebo simulation only: worlds and plugins, depends on `hyper_control`.
- `src/planning/hyper_planner` (ROS package `hyper_planner`) — the current behavior/planning stack: costmap, hybrid A* planner, behavior supervisor, controller. Apache-2.0. Its `waypoint_tools/` subfolder is currently empty; `course.yaml` and its authoring scripts live under `src/waypoint` below, and `config/parking_params.yaml` / `launch/parking_system_cpp.launch.py` already default `course_yaml` there — no `course_yaml:=` override needed unless you're pointing at a different course file (e.g. sim vs. practice vs. competition).
- `src/waypoint` — **not a ROS 2 package** (no `package.xml`), but the live location of `course.yaml` (the event/path data `hyper_planner`'s behavior supervisor reads at startup by default) plus the scripts used to author it (`waypoint_recorder_node.py`, `generate_arc_path.py`, `waypoint_view_node.py` — not `ros2 run` targets, run directly with `python3`).

No test suite and no CI are set up in this repo yet.

## Build & run

- ROS 2 Humble, Gazebo via `ros_gz`.
- Build: `colcon build` (or `--packages-select <pkg>`), then `source install/setup.bash` in every new terminal.
- `ros2 launch hyper_launch simulation.launch.py` launches the full stack (sim, odometry, perception, behavior) as one process tree, staggered via `TimerAction`; `Ctrl-C` tears it all down. `src/launcher/hyper_launch` also has one launch file per stage (`sim.launch.py`, `odometry.launch.py`, `perception.launch.py`, `behavior.launch.py`) for running a stage standalone — each is a thin `IncludeLaunchDescription` wrapper around the underlying package's own launch file (`hyper_gazebo/vehicle.launch.py`, `hyper_localization/odometry.launch.py`, `hyper_object_detection/perception.launch.py`, `hyper_planner/parking_system_cpp.launch.py`), which still work directly via their own packages too.
- See `README.md` for the full command and topic reference — don't duplicate it here.

## Branch structure

```
main          <- stable, PR-only, no direct pushes
  |- perception, control, planning, dev
```

## Working agreements

- Never push directly to `main` — all changes land via PR.
- ROS 2/C++ changes aren't verified by reading code alone — rebuild, re-source `install/setup.bash`, and relaunch the relevant node before calling something fixed.
