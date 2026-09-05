# hyper_planner — notes for Claude

Read `README.md` in this directory first; it is the reference for the step queue,
services and topics. Only the things it doesn't say belong here.

- **Parameters**: `src/mission_manager_parameters.yaml` is the source of truth for
  `mission_manager_node`'s parameters. `generate_parameter_library` compiles it into a header
  under `build/hyper_planner/` — never edit that generated header, and never hand-declare a
  parameter in the `.cpp` that the yaml already owns. Changing the yaml requires a rebuild.
- **`follow_path_client_node` no longer exists** in `src/` (only `mission_manager_node.cpp` and
  `cmd_vel_to_ackermann_node.cpp` do). The README's legacy section and the launch-file comments
  are historical — don't restore the node, and don't route new work through it.
- **Stopping is a timeout, not a command**: `cmd_vel_to_ackermann_node`'s `input_timeout`
  watchdog is how a `stop` step halts the car. Anything that keeps publishing `/cmd_vel` during
  a stop silently defeats it.
- Verify changes by relaunching `mission.launch.py` against `config/simple.yaml` — it's one goal,
  no signals or parking, so a regression shows up immediately.
