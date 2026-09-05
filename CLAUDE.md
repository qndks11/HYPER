# CLAUDE.md

## Project

HYPER is a ROS 2 (Humble) autonomous-driving stack built for **HL FMA (Future Mobility Award) 2026**, the **[1/5] category** ("aMAP Innovator Championship") — a university-division competition run by HL Mando, HL Klemove, and Halla University, using 1/5-scale autonomous vehicles.

## Build & run

- ROS 2 Humble, Gazebo via `ros_gz`.
- Build: `colcon build` (or `--packages-select <pkg>`), then `source install/setup.bash` in every new terminal.
- `ros2 launch hyper_launch simulation.launch.py` launches the full stack (sim, odometry, perception, behavior) as one process tree, staggered via `TimerAction`; `Ctrl-C` tears it all down. `src/launcher/hyper_launch` also has one launch file per stage (`sim.launch.py`, `odometry.launch.py`, `perception.launch.py`, `behavior.launch.py`) for running a stage standalone — each is a thin `IncludeLaunchDescription` wrapper around the underlying package's own launch file (`hyper_gazebo/vehicle.launch.py`, `hyper_localization/odometry.launch.py`, `hyper_object_detection/perception.launch.py`, `hyper_planner/parking_system_cpp.launch.py`), which still work directly via their own packages too.
- See `README.md` for the full command and topic reference — don't duplicate it here.
- GUI nodes (`rviz2`, `hyper_rqt`'s panel, anything opening an OpenCV/Qt window) crash instantly
  when launched from the VSCode integrated terminal on this machine — the snap injects a `GTK_PATH`
  that pulls in a confined `libpthread.so.0`. Prefix with
  `env -u GTK_PATH -u GTK_EXE_PREFIX -u GDK_PIXBUF_MODULE_FILE -u GDK_PIXBUF_MODULEDIR`.
  This applies to `use_rviz:=true` / `use_panel:=true` and to `real.launch.py` (rviz on by default).

## Where to look

Every package has its own `README.md`, and they are kept current — **read the README of the one
package you're touching before its sources**, instead of grepping the tree. One line each:

| Package | README | Owns |
| --- | --- | --- |
| `hyper_launch` | `src/launcher/hyper_launch/README.md` | whole-stack + per-stage launch files |
| `hyper_planner` | `src/planning/hyper_planner/README.md` | mission step queue, nav2 `follow_path`, cmd_vel→Ackermann |
| `hyper_costmap_plugins` | `src/planning/hyper_costmap_plugins/README.md` | nav2 costmap layers (`DrivableAreaLayer`) |
| `hyper_waypoint` | `src/planning/hyper_waypoint/README.md` | waypoint CSV recorder off `odometry/filtered_map` |
| `hyper_localization` | `src/localization/hyper_localization/README.md` | dual EKF + `navsat_transform`, datum config |
| `hyper_lane_detection` | `src/perception/hyper_lane_detection/README.md` | lane/stopline from camera, BEV, drivable area |
| `hyper_object_detection` | `src/perception/hyper_object_detection/README.md` | YOLO traffic-light/sign → `/perception/sign` |
| `hyper_control` | `src/control/hyper_control/README.md` | vehicle URDF, low-level control, joystick teleop |
| `hyper_gazebo` | `src/simulator/hyper_gazebo/README.md` | sim-only: world, spawn, gz↔ROS bridge |
| `hyper_interface` | `src/interface/hyper_interface/README.md` | real-car ROS↔Arduino serial bridge |
| `hyper_rqt` | `src/tools/hyper_rqt/README.md` | rqt panel of the services you actually call |
| `hyper_rtk`, `hyper_ebimu`, `hyper_camera`, `hyper_lidar`, `ntrip_client`, `ublox`, `witmotion_ros2` | `src/sensing/*/README.md` | real-car sensor drivers — see `src/sensing/CLAUDE.md` |

## Branch structure

```
main          <- stable, PR-only, no direct pushes
  |- perception, control, planning, dev
```

## Working agreements

- Never push directly to `main` — all changes land via PR.
- ROS 2/C++ changes aren't verified by reading code alone — rebuild, re-source `install/setup.bash`, and relaunch the relevant node before calling something fixed.
- Kill every node, launch tree or sim you started to verify something, as soon as you're done with it. A leftover ROS process is not idle: it keeps publishing on the same topics, so the next run silently picks up a duplicate publisher, a stale TF, or an old build's output and "verifies" the wrong thing. Check with `pgrep -af <node>` before reporting a result, not after.
