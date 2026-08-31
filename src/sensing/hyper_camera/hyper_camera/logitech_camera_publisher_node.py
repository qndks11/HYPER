#!/usr/bin/env python3

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class LogitechCameraPublisher(Node):
    """
    Logitech C920 카메라를 열어 원본 프레임을 image_raw로 발행한다.

    object_detection_node가 예전에 (input_backend:=direct_usb로) 직접 열던 카메라를 대신
    소유한다. rclpy에는 rclcpp의 intra-process 통신 같은 zero-copy 경로가 없으므로, 이 노드는
    일반 ROS 토픽으로만 발행한다 -- object_detection_node는 Gazebo의 시뮬레이션 브리지를
    구독할 때와 똑같은 코드 경로(image_callback)로 이 토픽을 구독한다.
    """

    def __init__(self):
        super().__init__('logitech_camera_publisher')

        self.declare_parameter('video_device', '/dev/video_logitech')
        # 640x360, not the C920's 1280x720. object_detection_node letterboxes every frame into
        # the YOLO model's own input size anyway, so the extra pixels buy nothing downstream and
        # cost USB bandwidth, MJPEG decode and battery. 16:9 is kept so the framing -- and with it
        # how large a traffic light is in the model's input -- does not change.
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 360)
        self.declare_parameter('framerate', 30.0)
        self.declare_parameter('frame_id', 'camera_object')

        video_device = self.get_parameter('video_device').value
        image_width = int(self.get_parameter('image_width').value)
        image_height = int(self.get_parameter('image_height').value)
        framerate = float(self.get_parameter('framerate').value)
        self.frame_id = self.get_parameter('frame_id').value

        self.bridge = CvBridge()

        self.capture = cv2.VideoCapture(video_device, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            raise RuntimeError(
                f"logitech_camera_publisher: failed to open camera device '{video_device}'"
            )

        # MJPEG in, decoded to BGR8 out -- OpenCV's V4L2 backend does the JPEG decode internally
        # on read() (CAP_PROP_CONVERT_RGB stays at its default True).
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, image_width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, image_height)
        self.capture.set(cv2.CAP_PROP_FPS, framerate)

        # Depth 10, matching object_detection_node's subscription QoS on the other end.
        self.image_publisher = self.create_publisher(Image, 'image_raw', 10)

        self.capture_timer = self.create_timer(
            1.0 / framerate,
            self.capture_timer_callback
        )

        self.get_logger().info(
            f"logitech_camera_publisher: opened '{video_device}' "
            f'({image_width}x{image_height} @ {framerate:.0f} fps), '
            f"publishing on '{self.image_publisher.topic_name}'"
        )

    def capture_timer_callback(self):
        ok, frame = self.capture.read()
        if not ok or frame is None:
            # Skip this tick rather than aborting the node over what may be a transient USB
            # glitch -- same treatment as hyper_camera's ElpCameraCapture::read().
            self.get_logger().error(
                'logitech_camera_publisher: camera stream read failed (device disconnected?)'
            )
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.image_publisher.publish(msg)

    def destroy_node(self):
        self.capture.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = LogitechCameraPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
