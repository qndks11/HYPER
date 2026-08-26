# CLAUDE.md

## Project

HYPER is a ROS 2 (Humble) autonomous-driving stack built for **HL FMA (Future Mobility Award) 2026**, the **[1/5] category** ("aMAP Innovator Championship") — a university-division competition run by HL Mando, HL Klemove, and Halla University, using 1/5-scale autonomous vehicles.

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
- Kill every node, launch tree or sim you started to verify something, as soon as you're done with it. A leftover ROS process is not idle: it keeps publishing on the same topics, so the next run silently picks up a duplicate publisher, a stale TF, or an old build's output and "verifies" the wrong thing. Check with `pgrep -af <node>` before reporting a result, not after.
