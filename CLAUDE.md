# CLAUDE.md

## Project

HYPER is a ROS 2 (Humble) autonomous-driving stack built for **HL FMA (Future Mobility Award) 2026**, the **[1/5] category** ("aMAP Innovator Championship") — a university-division competition run by HL Mando, HL Klemove, and Halla University, using 1/5-scale autonomous vehicles.

## Repo layout

Colcon workspace rooted here (`~/HYPER`). `build/`, `install/`, `log/` are gitignored build artifacts.

- `src/auto_vehicle` — core package: Ackermann vehicle control, joystick teleop, lane/stopline detection, YOLO-based object/traffic-light detection. MIT license.
- `src/auto_vehicle_gazebo` — Gazebo simulation only: worlds and plugins, depends on `auto_vehicle`.
- `src/total/parking_cpp_t8_bundle` (ROS package `parking_cpp`) — the current behavior/planning stack: costmap, hybrid A* planner, behavior supervisor, controller. Apache-2.0.
- `src/waypoint` — **legacy, not a ROS 2 package** (no `package.xml`). Superseded by `parking_cpp`. Don't build on or edit this unless explicitly asked.

No test suite and no CI are set up in this repo yet.

## Build & run

- ROS 2 Humble, Gazebo via `ros_gz`.
- Build: `colcon build` (or `--packages-select <pkg>`), then `source install/setup.bash` in every new terminal.
- `./run_all.sh` launches the full stack (sim, odometry, perception, behavior) across four terminals; `./run_all.sh stop` tears it down.
- See `README.md` for the full command and topic reference — don't duplicate it here.

## Branch structure

```
main          <- stable, PR-only, no direct pushes
  |- perception, control, planning, dev
```

## Working agreements

- Never push directly to `main` — all changes land via PR.
- Stay within the current branch's domain (e.g., on `perception`, don't edit `control`/`planning` code unless explicitly asked).
- ROS 2/C++ changes aren't verified by reading code alone — rebuild, re-source `install/setup.bash`, and relaunch the relevant node before calling something fixed.
