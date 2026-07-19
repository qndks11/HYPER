#!/usr/bin/env python3
# ros2 run auto_vehicle image_saver --ros-args -r /image_raw:=/camera/image_raw
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageSaver(Node):

    def __init__(self):
        super().__init__('image_saver')

        self.declare_parameter('save_dir', os.path.expanduser('~/image_captures'))
        self.declare_parameter('interval_sec', 4.0)

        self.save_dir = self.get_parameter('save_dir').value
        self.interval_sec = self.get_parameter('interval_sec').value
        os.makedirs(self.save_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.latest_frame = None
        self.saved_count = 0

        self.image_subscriber = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)

        self.create_timer(self.interval_sec, self.save_latest_frame)

        self.get_logger().info(
            f'image_saver started: saving to {self.save_dir} every {self.interval_sec}s')

    def image_callback(self, msg: Image):
        self.latest_frame = msg

    def save_latest_frame(self):
        if self.latest_frame is None:
            self.get_logger().warn('No image received yet, skipping save')
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(self.latest_frame, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge exception: {e}')
            return

        timestamp = time.strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(self.save_dir, f'{timestamp}_{self.saved_count:05d}.jpg')
        cv2.imwrite(filename, frame)
        self.saved_count += 1
        self.get_logger().info(f'Saved {filename}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()