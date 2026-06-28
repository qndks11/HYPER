import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from vision_msgs.msg import Detection2DArray
import cv2
import numpy as np
from cv_bridge import CvBridge


class Visualizer(Node):
    """Displays camera feed with lane and sign detection overlays via OpenCV."""

    def __init__(self):
        super().__init__('visualizer')
        self.bridge = CvBridge()

        self._latest_frame = None
        self._lane_data = None      # [offset, heading_err, valid]
        self._detections = []

        self.create_subscription(Image, '/image_raw', self._cb_image, 10)
        self.create_subscription(Float64MultiArray, '/lane/center', self._cb_lane, 10)
        self.create_subscription(Detection2DArray, '/perception/sign', self._cb_sign, 10)

        # 30 Hz display timer
        self.create_timer(1.0 / 30.0, self._render)
        self.get_logger().info('Visualizer started — press Q in the window to quit')

    def _cb_image(self, msg: Image):
        self._latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _cb_lane(self, msg: Float64MultiArray):
        if len(msg.data) >= 3:
            self._lane_data = msg.data

    def _cb_sign(self, msg: Detection2DArray):
        self._detections = msg.detections

    def _render(self):
        if self._latest_frame is None:
            return

        frame = self._latest_frame.copy()
        h, w = frame.shape[:2]

        # --- lane overlay ---
        if self._lane_data is not None:
            offset, heading_err, valid = self._lane_data[:3]
            color = (0, 255, 0) if valid else (0, 0, 255)
            label = f'offset={offset:+.3f}  heading={heading_err:+.3f}'
            cv2.putText(frame, label, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            # draw centre indicator bar
            bar_y = h - 20
            bar_w = w // 2
            cx = int(w / 2 + offset * bar_w)
            cv2.line(frame, (w // 4, bar_y), (3 * w // 4, bar_y), (200, 200, 200), 2)
            cv2.circle(frame, (cx, bar_y), 8, color, -1)

        # --- detection boxes ---
        for det in self._detections:
            bx = int(det.bbox.center.position.x)
            by = int(det.bbox.center.position.y)
            bw2 = int(det.bbox.size_x / 2)
            bh2 = int(det.bbox.size_y / 2)
            x1, y1, x2, y2 = bx - bw2, by - bh2, bx + bw2, by + bh2

            label = ''
            score = 0.0
            if det.results:
                label = det.results[0].hypothesis.class_id
                score = det.results[0].hypothesis.score

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 255), 2)
            cv2.putText(frame, f'{label} {score:.2f}', (x1, max(y1 - 6, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)

        cv2.imshow('HYPER Perception', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('Visualizer window closed by user')
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Visualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
