#!/usr/bin/env python3
"""
waypoint_recorder.py

Records a dense sequence of absolute GPS waypoints (lat/lon) for
hyper_waypoint_tracker. Run directly with python3 (not a ros2 entry point),
matching the src/waypoint/waypoint_recorder_node.py convention.

Usage:
    source install/setup.bash
    python3 src/planning/hyper_waypoint_tracker/scripts/waypoint_recorder.py

Commands (std_msgs/String on /waypoint_tracker/cmd):
    start   - begin/resume recording (drive to the start line first, then start)
    stop    - pause recording
    clear   - discard everything recorded so far
    save    - write the recorded waypoints to disk

    ros2 topic pub --once /waypoint_tracker/cmd std_msgs/msg/String "{data: 'start'}"

A clean Ctrl-C also auto-saves.
"""

import math
import sys

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

EARTH_RADIUS_M = 6371000.0


def haversine_m(lat1, lon1, lat2, lon2):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
    return 2.0 * EARTH_RADIUS_M * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


class WaypointRecorder(Node):

    def __init__(self):
        super().__init__('waypoint_recorder')

        self.min_spacing_m = float(self.declare_parameter('min_spacing_m', 0.5).value)
        self.output_path = str(self.declare_parameter(
            'output_path',
            '~/HYPER/src/planning/hyper_waypoint_tracker/waypoints/waypoints.yaml',
        ).value)

        self.recording = False
        self.waypoints = []  # list of (lat, lon)

        self.create_subscription(NavSatFix, '/gps/fix', self.on_gps, 10)
        self.create_subscription(String, '/waypoint_tracker/cmd', self.on_cmd, 10)

        self.get_logger().info(
            "waypoint_recorder ready. Publish to /waypoint_tracker/cmd: "
            "start | stop | clear | save")

    def on_gps(self, msg: NavSatFix):
        if not self.recording:
            return
        if self.waypoints:
            last_lat, last_lon = self.waypoints[-1]
            if haversine_m(last_lat, last_lon, msg.latitude, msg.longitude) < self.min_spacing_m:
                return
        self.waypoints.append((msg.latitude, msg.longitude))
        self.get_logger().info(
            f"recorded waypoint {len(self.waypoints)}: "
            f"({msg.latitude:.7f}, {msg.longitude:.7f})")

    def on_cmd(self, msg: String):
        cmd = msg.data.strip()
        if cmd == 'start':
            self.recording = True
            self.get_logger().info("recording started")
        elif cmd == 'stop':
            self.recording = False
            self.get_logger().info(f"recording stopped ({len(self.waypoints)} waypoints so far)")
        elif cmd == 'clear':
            self.waypoints.clear()
            self.get_logger().info("cleared recorded waypoints")
        elif cmd == 'save':
            self.save()
        else:
            self.get_logger().warn(f"unknown command: '{cmd}'")

    def save(self):
        import os
        path = os.path.expanduser(self.output_path)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        data = {
            'waypoints': [
                {'latitude': lat, 'longitude': lon} for lat, lon in self.waypoints
            ],
        }
        with open(path, 'w') as f:
            yaml.safe_dump(data, f, sort_keys=False)
        self.get_logger().info(f"saved {len(self.waypoints)} waypoints to {path}")


def main():
    rclpy.init()
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
