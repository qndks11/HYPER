#!/usr/bin/env python3

import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

WINDOW_NAME = 'Object Detection'


class ObjectDetection(Node):

    def __init__(self):
        super().__init__('object_detection')

        self.declare_parameter('model_path', 'best.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('detection_frequency', 5)
        self.declare_parameter('save_dir', os.path.expanduser('~/traffic_dataset'))
        self.n_frames = 0

        model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.detection_frequency = self.get_parameter('detection_frequency').value
        self.save_dir = self.get_parameter('save_dir').value
        os.makedirs(self.save_dir, exist_ok=True)

        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.saved_count = 0

        self.image_subscriber = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)

        self.traffic_light_publisher = self.create_publisher(
            String, '/traffic_light/state', 10)

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

        self.get_logger().info('ObjectDetection started')

    def image_callback(self, msg: Image):
        self.n_frames += 1

        if self.n_frames % self.detection_frequency != 0:
           return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge exception: {e}')
            return

        results = self.model(frame, conf=self.confidence_threshold, verbose=False)
        annotated_frame = results[0].plot()

        self.publish_middle_traffic_light(results[0], frame.shape[1])

        cv2.imshow(WINDOW_NAME, annotated_frame)
        cv2.waitKey(1)
        # key = cv2.waitKey(1) & 0xFF
        # if key == ord('s'):
        #    self.saved_count += 1
        #    filename = os.path.join(self.save_dir, f'frame_{self.saved_count:05d}.jpg')
        #    cv2.imwrite(filename, frame)
        #    self.get_logger().info(f'Saved {filename}')

    def publish_middle_traffic_light(self, result, image_width):
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return

        left_bound = image_width * 0.25
        right_bound = image_width * 0.75
        image_center = image_width / 2.0

        best_box = None
        best_key = None
        for box in boxes:
            x1, _, x2, _ = box.xyxy[0].tolist()
            box_center = (x1 + x2) / 2.0
            if not (left_bound <= box_center <= right_bound):
                continue

            distance_to_center = abs(box_center - image_center)
            confidence = float(box.conf[0])
            # Closest to center wins; ties broken by higher confidence.
            key = (distance_to_center, -confidence)
            if best_key is None or key < best_key:
                best_key = key
                best_box = box

        if best_box is None:
            return

        class_name = self.model.names[int(best_box.cls[0])]
        confidence = float(best_box.conf[0])

        msg = String()
        msg.data = class_name
        self.traffic_light_publisher.publish(msg)

        self.get_logger().info(f'Detected traffic light: {class_name} ({confidence:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
