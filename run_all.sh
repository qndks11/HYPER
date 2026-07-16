#!/usr/bin/env bash
# Opens 5 terminals and runs the full autonomous-driving stack, one node group per terminal.
# Terminal 1: Gazebo sim + vehicle spawn + low-level vehicle controller
# Terminal 2: localization (dual EKF + navsat_transform)
# Terminal 3: perception (lane/stopline detection + traffic light detector)
# Terminal 4: behavior/decision node
# Terminal 5: controller

set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# If this script is run from a terminal opened by the VS Code snap, the shell
# inherits GTK_PATH/GDK_PIXBUF_MODULEDIR/LOCPATH/etc. pointing into
# /snap/code/.../usr/lib. gnome-terminal.real then loads GTK modules built
# against the core20 snap's glibc, which crashes with a libpthread symbol
# mismatch. Strip the snap-injected vars so gnome-terminal gets a clean,
# host environment instead.
run_in_terminal() {
    local title="$1"
    local command="$2"

    env -u GTK_PATH -u GTK_EXE_PREFIX -u GTK_IM_MODULE_FILE \
        -u GDK_PIXBUF_MODULE_FILE -u GDK_PIXBUF_MODULEDIR \
        -u GIO_MODULE_DIR -u GSETTINGS_SCHEMA_DIR -u LOCPATH \
        -u SNAP_LIBRARY_PATH -u XDG_DATA_DIRS -u XDG_CONFIG_DIRS \
        gnome-terminal --title="$title" -- bash -c "
        cd '$WORKSPACE_DIR'
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        echo '=== $title ==='
        $command
        exec bash
    "
}

run_in_terminal "1-sim"        "ros2 launch auto_vehicle_gazebo vehicle.launch.py"
sleep 5

run_in_terminal "2-odometry"   "ros2 launch auto_vehicle odometry.launch.py"
sleep 2

run_in_terminal "3-perception" "ros2 launch auto_vehicle perception.launch.py"
sleep 2

run_in_terminal "4-behavior"   "python3 src/waypoint/behavior.py"
sleep 1

run_in_terminal "5-controller" "python3 src/waypoint/controller.py"
