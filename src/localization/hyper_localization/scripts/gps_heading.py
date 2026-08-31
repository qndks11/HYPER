#!/usr/bin/env python3
"""Absolute yaw for ekf_global from GPS course over ground.

Why this node exists
--------------------
The WitMotion WT901BLE's yaw is not trustworthy as an absolute heading: it runs
in 6-axis mode (no usable magnetometer fusion), so its zero is wherever the
sensor happened to be powered on and it drifts ~0.5 deg/min from there. Feeding
that into ekf_global as absolute yaw slowly rotates the whole map frame, and
fights the RTK position solution.

So ekf_global no longer fuses IMU yaw at all (see dual_ekf_navsat.yaml:
imu0_config index 5 is false in *both* filters now). Yaw is instead:

  * *unknown at boot* -- there is no surveyed constant and no magnetometer, so
    until something below establishes it, ekf_global's yaw is just the EKF's
    initial state (0) carried by the z gyro. The map frame heading is
    meaningless in that window; nothing should drive autonomously in it.
  * *established* either by the first accepted GPS course over ground (this
    node, once the car is moving fast/straight enough) or by an external
    one-shot ``/imu/heading`` injection -- the "roll 0.5 m" yaw calibration in
    gps_accuracy_gui, which drives the car a short way and measures yaw from
    the GPS displacement, then publishes it here with a covariance from that
    baseline. That calibration is the deliberate replacement for the old
    surveyed ``initial_heading_deg`` constant.
  * *propagated* by the IMU's z gyro, which both EKFs already fuse;
  * *corrected* thereafter by GPS course over ground whenever the car is moving
    fast enough for the course to mean something.

That leaves no magnetometer anywhere in the loop. navsat_transform is already
magnetometer-free: with ``wait_for_datum: true`` robot_localization never even
subscribes to /imu (navsat_transform.cpp:186), it pins the map frame using the
datum's own heading.

A GPS-denied run therefore has no yaw origin at all -- run the GUI yaw
calibration first (it only needs GPS for the ~0.5 m roll) or accept that yaw is
pure gyro integration off an arbitrary zero.

Output
------
``/imu/heading`` (sensor_msgs/Imu) carrying yaw only. ekf_global takes just the
yaw element from it; roll/pitch are published with a huge covariance and the
gyro/accel fields are flagged absent (covariance[0] = -1, REP-145). Each
accepted course is published exactly once, as it arrives; between them (and any
time the car is too slow for a meaningful course) nothing is published and the
gyro carries yaw on its own.

Course sources, in priority order:

  * ``/ublox_gps_node/navpvt`` (ublox_msgs/NavPVT): ``heading`` is course over
    ground and ``head_acc`` is the receiver's own accuracy estimate for it,
    which we turn straight into the measurement covariance. Preferred whenever
    it is arriving.
  * ``/gps/fix`` differencing: course from the displacement between the current
    fix and the most recent earlier fix at least ``min_fix_displacement`` away
    (within a ``max_baseline_age`` window), with covariance from angular
    uncertainty over that baseline. Baselines overlap, so a course comes out at
    the fix rate rather than once per ``min_fix_displacement`` travelled. Used
    when NavPVT has been silent for ``navpvt_timeout`` -- which is the case in
    Gazebo, where there is no u-blox driver at all.

Both are course over *ground*, so they point backwards when the car reverses;
``/odom``'s signed twist.linear.x (the Arduino's measured wheel speed) supplies
both the speed gate and the reverse flip.
"""

import math
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix

try:
    from ublox_msgs.msg import NavPVT
except ImportError:  # pragma: no cover - sim installs may not build ublox_msgs
    NavPVT = None

_EARTH_RADIUS_M = 6378137.0
# Roll/pitch are published only because an Imu message has to carry a full
# quaternion; ekf_global does not fuse them from this topic.
_UNUSED_VARIANCE = 1e6


def _yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class GpsHeading(Node):
    def __init__(self):
        super().__init__('gps_heading')

        p = self.declare_parameter
        # Speed gate. Below this the course over ground is dominated by GPS
        # noise rather than by where the car is pointing, so we publish nothing
        # and let the gyro carry yaw.
        self.min_speed = p('min_speed', 0.5).value
        self.max_head_acc = math.radians(p('max_head_acc_deg', 25.0).value)
        # Floor on the reported accuracy. u-blox can claim a sub-degree course
        # that is still off by the car's own slip/crab angle, and believing it
        # makes ekf_global snap.
        self.min_stddev = math.radians(p('min_heading_stddev_deg', 5.0).value)
        # Baseline needed before two consecutive fixes define a course.
        self.min_fix_displacement = p('min_fix_displacement', 0.5).value
        self.fix_position_stddev = p('fix_position_stddev', 0.3).value
        # How far back the fix window reaches. Long enough that a slow car
        # still travels min_fix_displacement within it, short enough that a
        # baseline cannot span a turn.
        self.max_baseline_age = p('max_baseline_age', 3.0).value
        self.navpvt_timeout = p('navpvt_timeout', 3.0).value
        self.odom_timeout = p('odom_timeout', 1.0).value
        self.frame_id = p('frame_id', 'body_link').value

        navpvt_topic = p('navpvt_topic', '/ublox_gps_node/navpvt').value
        fix_topic = p('fix_topic', 'gps/fix').value
        odom_topic = p('odom_topic', 'odom').value
        output_topic = p('output_topic', 'imu/heading').value

        self._established = False   # has any absolute yaw been published yet
        self._odom_vx = None
        self._odom_stamp = None
        self._navpvt_stamp = None
        # Recent fixes, oldest first, trimmed to max_baseline_age. Each new
        # fix takes its course from the newest of these far enough behind it.
        self._fixes = deque()

        self.pub = self.create_publisher(Imu, output_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(NavSatFix, fix_topic, self._on_fix, 10)
        if NavPVT is not None:
            self.create_subscription(NavPVT, navpvt_topic, self._on_navpvt, 10)
        else:
            self.get_logger().warn(
                'ublox_msgs not available -- falling back to /gps/fix differencing')

        self.get_logger().info(
            'gps_heading: yaw unknown until the first GPS course (or a one-shot '
            f'{output_topic} injection); course sources {navpvt_topic} then {fix_topic}')

    # ---------------------------------------------------------------- inputs
    def _on_odom(self, msg):
        self._odom_vx = msg.twist.twist.linear.x
        self._odom_stamp = self.get_clock().now()

    def _speed(self):
        """Signed wheel speed, or None when /odom is stale or absent."""
        if self._odom_vx is None or self._odom_stamp is None:
            return None
        age = (self.get_clock().now() - self._odom_stamp).nanoseconds / 1e9
        return None if age > self.odom_timeout else self._odom_vx

    def _travel_to_heading(self, yaw_of_travel):
        """Direction of travel (ENU yaw) -> the direction the car is pointing.

        Course over ground points backwards when reversing, so flip it when the
        wheels say we are rolling backwards -- otherwise every reverse
        manoeuvre injects a 180-degree yaw error.
        """
        speed = self._speed()
        if speed is not None and speed < 0.0:
            yaw_of_travel += math.pi
        return _wrap(yaw_of_travel)

    def _moving(self, gps_speed):
        """True when the car is fast enough for course over ground to mean
        something. Prefers the wheel encoder, falls back to GPS ground speed."""
        speed = self._speed()
        if speed is None:
            speed = gps_speed
        return speed is not None and abs(speed) >= self.min_speed

    def _on_navpvt(self, msg):
        self._navpvt_stamp = self.get_clock().now()

        if not (msg.flags & NavPVT.FLAGS_GNSS_FIX_OK):
            return
        if not self._moving(msg.g_speed / 1000.0):
            return

        head_acc = math.radians(msg.head_acc * 1e-5)
        if head_acc > self.max_head_acc:
            return

        # NavPVT.heading is course over ground the compass way (0 = North,
        # clockwise); ENU yaw is 0 = East, counter-clockwise.
        self._publish(
            self._travel_to_heading(math.pi / 2.0 - math.radians(msg.heading * 1e-5)),
            max(head_acc, self.min_stddev))

    def _on_fix(self, msg):
        if msg.status.status < 0 or math.isnan(msg.latitude) or math.isnan(msg.longitude):
            self._fixes.clear()
            return

        now = self.get_clock().now()

        # NavPVT is the better source whenever it is alive; only difference
        # fixes once it has gone quiet (or never existed, as in simulation).
        if self._navpvt_stamp is not None:
            age = (now - self._navpvt_stamp).nanoseconds / 1e9
            if age <= self.navpvt_timeout:
                self._fixes.clear()
                return
        if not self._moving(None):
            # Too slow for a course, and the car may be turning in place --
            # nothing measured from before the stop is a usable baseline.
            self._fixes.clear()
            return

        # Sliding window rather than a single baseline that resets each time it
        # is consumed. Pairing every fix against an *earlier* one means a course
        # comes out at the full fix rate instead of once per
        # min_fix_displacement travelled -- and pairing against the newest fix
        # that is still far enough away keeps each baseline the shortest one
        # that clears the accuracy floor, so it spans as little of a turn as
        # possible.
        self._fixes.append((msg.latitude, msg.longitude, now))
        while self._fixes and (now - self._fixes[0][2]).nanoseconds / 1e9 > self.max_baseline_age:
            self._fixes.popleft()

        lat = math.radians(msg.latitude)
        cos_lat = math.cos(lat)
        for prev_lat, prev_lon, _ in reversed(list(self._fixes)[:-1]):
            east = math.radians(msg.longitude - prev_lon) * _EARTH_RADIUS_M * cos_lat
            north = math.radians(msg.latitude - prev_lat) * _EARTH_RADIUS_M
            displacement = math.hypot(east, north)
            if displacement < self.min_fix_displacement:
                continue

            # Position noise perpendicular to a baseline of `displacement`
            # metres turns into this much angular noise.
            stddev = math.atan2(self.fix_position_stddev, displacement)
            if stddev > self.max_head_acc:
                return
            self._publish(
                self._travel_to_heading(math.atan2(north, east)),
                max(stddev, self.min_stddev))
            return

    # --------------------------------------------------------------- outputs
    def _publish(self, yaw, stddev):
        if not self._established:
            self._established = True
            self.get_logger().info(
                f'first GPS course accepted ({math.degrees(yaw):.1f} deg ENU) -- '
                'yaw established, now gyro + GPS course')

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        (msg.orientation.x, msg.orientation.y,
         msg.orientation.z, msg.orientation.w) = _yaw_to_quat(yaw)
        msg.orientation_covariance = [
            _UNUSED_VARIANCE, 0.0, 0.0,
            0.0, _UNUSED_VARIANCE, 0.0,
            0.0, 0.0, stddev ** 2,
        ]
        # REP-145: leading -1 means "this message carries no such data".
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = GpsHeading()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
