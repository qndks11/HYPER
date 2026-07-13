#!/usr/bin/env python3

import os
import select
import sys
import termios
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


def decay_towards_zero(value: float, rate: float) -> float:
    if abs(value) <= rate:
        return 0.0
    return value - rate * (1.0 if value > 0.0 else -1.0)


class KeyboardController(Node):

    def __init__(self, timer_period: float = 1e-2):
        super().__init__('keyboard_controller')

        # Declare the used parameters
        self.declare_parameter('max_steering_angle', 2.0)
<<<<<<< Updated upstream
        self.declare_parameter('max_velocity', 5.0)
        self.declare_parameter('steering_step', 0.02)
        self.declare_parameter('velocity_step', 0.2)
=======
        self.declare_parameter('max_velocity', 30.0)          # 15.0 -> 30.0, double top speed
        self.declare_parameter('steering_step', 0.02)
        self.declare_parameter('velocity_step', 6.0)           # 3.0 -> 6.0, double accel too
>>>>>>> Stashed changes
        self.declare_parameter('steering_center_rate', 0.05)
        self.declare_parameter('velocity_center_rate', 0.1)
        self.declare_parameter('key_hold_timeout', 0.5)
        self.declare_parameter('steering_smoothing_alpha', 0.006)  # 0.2 -> 0.08, noticeably softer

        # Get parameters on startup
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.steering_step = self.get_parameter('steering_step').value
        self.velocity_step = self.get_parameter('velocity_step').value
        self.steering_center_rate = self.get_parameter('steering_center_rate').value
        self.velocity_center_rate = self.get_parameter('velocity_center_rate').value
        self.key_hold_timeout = self.get_parameter('key_hold_timeout').value
        self.steering_smoothing_alpha = self.get_parameter('steering_smoothing_alpha').value

        # steering_target: what the key presses are asking for (steps immediately, like before)
        # steering_angle: what actually gets published, eased toward the target each tick
        self.steering_target = 0.0
        self.steering_angle = 0.0
        self.velocity = 0.0
        self.should_quit = False
        # Terminal auto-repeat only re-sends a held key every ~25-50ms, far slower
        # than our 10ms timer tick, so we can't tell "released" from "no repeat yet
        # this tick". Instead track when each key was last seen and treat it as
        # still held until key_hold_timeout has passed since then.
        self._last_seen = {'w': -1.0, 's': -1.0, 'a': -1.0, 'd': -1.0}

        # Publishers for /steering_angle and /velocity topics
        self.steering_angle_publisher = self.create_publisher(Float64, '/steering_angle', 1)
        self.velocity_publisher = self.create_publisher(Float64, '/velocity', 1)

        self._stdin_fd = sys.stdin.fileno()
        self._original_termios = termios.tcgetattr(self._stdin_fd)
        self._enable_raw_mode()
        self._print_instructions()

        # Timer to periodically read keyboard input and publish the desired angle and velocity
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def destroy_node(self):
        self._disable_raw_mode()
        super().destroy_node()

    def _enable_raw_mode(self):
        # Disable canonical input, echoing and signal generation so WASD keypresses can be
        # read as soon as they arrive, without waiting for a newline.
        raw = termios.tcgetattr(self._stdin_fd)
        raw[3] &= ~(termios.ICANON | termios.ECHO | termios.ISIG)
        raw[6][termios.VMIN] = 0
        raw[6][termios.VTIME] = 0
        termios.tcsetattr(self._stdin_fd, termios.TCSANOW, raw)

    def _disable_raw_mode(self):
        termios.tcsetattr(self._stdin_fd, termios.TCSANOW, self._original_termios)

    def _print_instructions(self):
        print(
            '\nKeyboard Controller\n'
            '--------------------\n'
            '  w : accelerate forward\n'
            '  s : accelerate backward\n'
            '  a : steer left\n'
            '  d : steer right\n'
            '  space : stop and center\n'
            '  q or Ctrl-C : quit\n\n'
            'Releasing a/d lets the steering ease back to center.\n'
            'Releasing w/s lets the velocity ease back to zero.\n'
        )

    def _read_keys(self) -> str:
        keys = ''
        while select.select([self._stdin_fd], [], [], 0)[0]:
            chunk = os.read(self._stdin_fd, 16)
            if not chunk:
                break
            keys += chunk.decode(errors='ignore')
        return keys

    def timer_callback(self):
        keys = self._read_keys()
        now = time.monotonic()

        for key in self._last_seen:
            if key in keys:
                self._last_seen[key] = now

        def held(key: str) -> bool:
            return (now - self._last_seen[key]) <= self.key_hold_timeout

        forward = held('w')
        backward = held('s')
        left = held('a')
        right = held('d')
        stop = ' ' in keys
        quit_ = 'q' in keys or '\x03' in keys

        if quit_:
            # Don't shut down here: tearing down the context from inside a callback
            # that the executor's spin loop is still running deadlocks the wait-set.
            # Flag it instead and let main()'s loop stop and tear down safely.
            self.should_quit = True
            return

        if stop:
            self.velocity = 0.0
            self.steering_target = 0.0
        else:
            if forward and not backward:
                self.velocity = min(self.velocity + self.velocity_step, self.max_velocity)
            elif backward and not forward:
                self.velocity = max(self.velocity - self.velocity_step, -self.max_velocity)
            else:
                self.velocity = decay_towards_zero(self.velocity, self.velocity_center_rate)

            if left and not right:
                self.steering_target = min(
                    self.steering_target + self.steering_step, self.max_steering_angle)
            elif right and not left:
                self.steering_target = max(
                    self.steering_target - self.steering_step, -self.max_steering_angle)
            else:
                self.steering_target = decay_towards_zero(
                    self.steering_target, self.steering_center_rate)

        # Ease the published steering angle toward the target instead of jumping to it.
        # Smaller steering_smoothing_alpha = smoother but slower to respond.
        self.steering_angle += (self.steering_target - self.steering_angle) * self.steering_smoothing_alpha

        # Publish the desired angle
        angle_msg = Float64()
        angle_msg.data = self.steering_angle
        self.steering_angle_publisher.publish(angle_msg)

        # Publish the desired velocity
        velocity_msg = Float64()
        velocity_msg.data = self.velocity
        self.velocity_publisher.publish(velocity_msg)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    try:
        while rclpy.ok() and not node.should_quit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()