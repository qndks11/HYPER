#!/usr/bin/env python3
"""요청 시 /camera/image_raw 프레임 한 장을 파일로 저장하는 서비스.

object_detection_node가 보는 것과 똑같은 카메라 토픽을 구독해 가장 최근 프레임을
들고 있다가, std_srvs/srv/Trigger 서비스(`~/save`)가 불리면 그 프레임을
save_dir(기본 ~/Pictures/object_detection)에 PNG로 떨어뜨립니다.

파일 이름은 자동으로 붙습니다: `objdet_YYYYmmdd_HHMMSS.png`. 같은 초에 두 번
부르거나 이미 그 이름이 있으면 `_001`, `_002` … 를 붙여 겹치지 않게 합니다.
"""
import os
from datetime import datetime

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


class ImageSaverService(Node):

    def __init__(self):
        super().__init__('image_saver_service')

        default_dir = os.path.join(
            os.path.expanduser('~'), 'Pictures', 'object_detection')
        self.declare_parameter('save_dir', default_dir)
        self.declare_parameter('prefix', 'objdet')

        self._bridge = CvBridge()
        self._last_frame = None

        self.create_subscription(
            Image, '/camera/image_raw', self._on_image, qos_profile_sensor_data)
        self.create_service(Trigger, '~/save', self._on_save)

        self.get_logger().info(
            f'saving to {self.get_parameter("save_dir").value} on ~/save')

    def _on_image(self, msg):
        self._last_frame = msg

    def _on_save(self, request, response):
        if self._last_frame is None:
            response.success = False
            response.message = '아직 /camera/image_raw 프레임을 못 받았습니다'
            return response

        try:
            frame = self._bridge.imgmsg_to_cv2(self._last_frame, 'bgr8')
        except Exception as exc:  # noqa: BLE001 -- 변환 실패를 응답으로 돌려줍니다
            response.success = False
            response.message = f'cv_bridge 변환 실패: {exc}'
            return response

        save_dir = self.get_parameter('save_dir').value
        prefix = self.get_parameter('prefix').value
        os.makedirs(save_dir, exist_ok=True)

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(save_dir, f'{prefix}_{stamp}.png')
        suffix = 1
        while os.path.exists(path):
            path = os.path.join(save_dir, f'{prefix}_{stamp}_{suffix:03d}.png')
            suffix += 1

        if not cv2.imwrite(path, frame):
            response.success = False
            response.message = f'cv2.imwrite 실패: {path}'
            return response

        response.success = True
        response.message = path
        self.get_logger().info(f'saved {path}')
        return response


def main():
    rclpy.init()
    node = ImageSaverService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
