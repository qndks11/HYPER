#!/usr/bin/env bash
# Opens 5 terminals and runs the full autonomous-driving stack, one node group per terminal.
# Terminal 1: Gazebo sim + vehicle spawn + low-level vehicle controller
# Terminal 2: localization (dual EKF + navsat_transform)
# Terminal 3: perception (lane/stopline detection + traffic light detector)
# Terminal 4: behavior/decision node
# Terminal 5: controller
#
# Run './run_all.sh stop' to kill every process the stack above starts (Gazebo,
# all ros2 launch trees, and their child nodes) and close the terminal windows
# themselves.

set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Each terminal's own shell PID is recorded here so 'stop' can kill the shell
# directly -- gnome-terminal closes a window when the command running in it
# exits, but stop_all otherwise only ever touches the ros2/Gazebo processes
# running underneath that shell, not the shell itself.
PID_DIR="${TMPDIR:-/tmp}/run_all-pids-$(id -u)"

# Patterns matching each terminal's launch command plus the Gazebo server
# process it starts underneath -- ros2 launch forwards SIGINT to its child
# nodes for a clean shutdown, but the Gazebo server process itself is started
# outside that tree and needs to be targeted directly, as do any nodes left
# behind if a launch tree was already killed uncleanly in a previous run.
STOP_PATTERNS=(
    "ros2 launch auto_vehicle_gazebo vehicle.launch.py"
    "ros2 launch auto_vehicle odometry.launch.py"
    "ros2 launch auto_vehicle perception.launch.py"
    "ros2 launch parking_cpp parking_system_cpp.launch.py"
    "ign gazebo"
    "gz sim"
    "robot_localization/ekf_node"
    "robot_localization/navsat_transform_node"
    "lib/auto_vehicle/lane_detection"
    "lib/auto_vehicle/object_detection"
    "lib/auto_vehicle/vehicle_controller"
    "lib/parking_cpp/"
)

stop_all() {
    echo "Stopping run_all.sh stack..."

    for pattern in "${STOP_PATTERNS[@]}"; do
        pkill -INT -f "$pattern" 2>/dev/null || true
    done

    sleep 3

    local remaining=0
    for pattern in "${STOP_PATTERNS[@]}"; do
        if pkill -KILL -f "$pattern" 2>/dev/null; then
            remaining=1
        fi
    done

    if [[ "$remaining" -eq 1 ]]; then
        echo "Some processes didn't shut down cleanly and were force-killed."
    else
        echo "All processes stopped cleanly."
    fi

    if [[ -d "$PID_DIR" ]]; then
        for pidfile in "$PID_DIR"/*.pid; do
            [[ -e "$pidfile" ]] || continue
            local pid
            pid="$(cat "$pidfile" 2>/dev/null || true)"
            if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
                kill -TERM "$pid" 2>/dev/null || true
            fi
            rm -f "$pidfile"
        done
        echo "Closed run_all.sh terminal windows."
    fi
}

if [[ "${1:-}" == "stop" ]]; then
    stop_all
    exit 0
fi

# If this script is run from a terminal opened by the VS Code snap, the shell
# inherits GTK_PATH/GDK_PIXBUF_MODULEDIR/LOCPATH/etc. pointing into
# /snap/code/.../usr/lib. gnome-terminal.real then loads GTK modules built
# against the core20 snap's glibc, which crashes with a libpthread symbol
# mismatch. Strip the snap-injected vars so gnome-terminal gets a clean,
# host environment instead.
run_in_terminal() {
    local title="$1"
    local command="$2"
    local pidfile="$PID_DIR/$title.pid"

    env -u GTK_PATH -u GTK_EXE_PREFIX -u GTK_IM_MODULE_FILE \
        -u GDK_PIXBUF_MODULE_FILE -u GDK_PIXBUF_MODULEDIR \
        -u GIO_MODULE_DIR -u GSETTINGS_SCHEMA_DIR -u LOCPATH \
        -u SNAP_LIBRARY_PATH -u XDG_DATA_DIRS -u XDG_CONFIG_DIRS \
        gnome-terminal --title="$title" -- bash -c "
        echo \$\$ > '$pidfile'
        cd '$WORKSPACE_DIR'
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        echo '=== $title ==='
        $command
        exec bash
    "
}

mkdir -p "$PID_DIR"

run_in_terminal "1-sim"        "ros2 launch auto_vehicle_gazebo vehicle.launch.py"
sleep 5

run_in_terminal "2-odometry"   "ros2 launch auto_vehicle odometry.launch.py"
sleep 2

run_in_terminal "3-perception" "ros2 launch auto_vehicle perception.launch.py"
sleep 2

run_in_terminal "4-behavior"   "ros2 launch parking_cpp parking_system_cpp.launch.py"
