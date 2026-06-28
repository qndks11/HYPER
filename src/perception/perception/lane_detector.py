import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
import cv2
import numpy as np
from cv_bridge import CvBridge


# ROI vertices as ratio of (height, width)
ROI_TOP_LEFT     = (0.6, 0.1)
ROI_TOP_RIGHT    = (0.6, 0.9)
ROI_BOTTOM_LEFT  = (1.0, 0.0)
ROI_BOTTOM_RIGHT = (1.0, 1.0)

BIRD_EYE_SIZE    = (400, 600)   # (width, height)
N_WINDOWS        = 9
MARGIN           = 60
MIN_PIX          = 30


class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/lane/center', 10)

        self.get_logger().info('LaneDetector started')

    # ---------- perspective ----------

    def _build_transform(self, h, w):
        src = np.float32([
            [w * ROI_TOP_LEFT[1],     h * ROI_TOP_LEFT[0]],
            [w * ROI_TOP_RIGHT[1],    h * ROI_TOP_RIGHT[0]],
            [w * ROI_BOTTOM_RIGHT[1], h * ROI_BOTTOM_RIGHT[0]],
            [w * ROI_BOTTOM_LEFT[1],  h * ROI_BOTTOM_LEFT[0]],
        ])
        bw, bh = BIRD_EYE_SIZE
        dst = np.float32([
            [0,   0],
            [bw,  0],
            [bw,  bh],
            [0,   bh],
        ])
        return cv2.getPerspectiveTransform(src, dst)

    def _bird_eye(self, img):
        h, w = img.shape[:2]
        M = self._build_transform(h, w)
        return cv2.warpPerspective(img, M, BIRD_EYE_SIZE)

    # ---------- binary mask ----------

    def _binary(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # white lane
        white_mask = cv2.inRange(hsv, (0, 0, 180), (180, 40, 255))

        # yellow lane
        yellow_mask = cv2.inRange(hsv, (15, 80, 80), (35, 255, 255))

        mask = cv2.bitwise_or(white_mask, yellow_mask)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return mask

    # ---------- sliding window ----------

    def _sliding_window(self, binary):
        bh, bw = binary.shape
        histogram = binary[bh // 2:, :].sum(axis=0)

        mid = bw // 2
        left_base  = int(np.argmax(histogram[:mid]))
        right_base = int(np.argmax(histogram[mid:]) + mid)

        win_h = bh // N_WINDOWS
        nonzero_y, nonzero_x = binary.nonzero()

        left_x, right_x = left_base, right_base
        left_pts, right_pts = [], []

        for win in range(N_WINDOWS):
            y_low  = bh - (win + 1) * win_h
            y_high = bh - win * win_h

            def _inwin(cx):
                return (nonzero_x >= cx - MARGIN) & (nonzero_x < cx + MARGIN) & \
                       (nonzero_y >= y_low) & (nonzero_y < y_high)

            left_idx  = np.where(_inwin(left_x))[0]
            right_idx = np.where(_inwin(right_x))[0]

            left_pts.extend(left_idx.tolist())
            right_pts.extend(right_idx.tolist())

            if len(left_idx) >= MIN_PIX:
                left_x  = int(nonzero_x[left_idx].mean())
            if len(right_idx) >= MIN_PIX:
                right_x = int(nonzero_x[right_idx].mean())

        return nonzero_x, nonzero_y, left_pts, right_pts, bw, bh

    # ---------- main callback ----------

    def image_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        warped = self._bird_eye(img)
        binary = self._binary(warped)

        nz_x, nz_y, left_pts, right_pts, bw, bh = self._sliding_window(binary)

        out = Float64MultiArray()
        valid = False
        offset = 0.0
        heading_err = 0.0

        if len(left_pts) >= MIN_PIX and len(right_pts) >= MIN_PIX:
            lx = nz_x[left_pts];  ly = nz_y[left_pts]
            rx = nz_x[right_pts]; ry = nz_y[right_pts]

            try:
                left_fit  = np.polyfit(ly, lx, 2)
                right_fit = np.polyfit(ry, rx, 2)

                y_eval = float(bh)
                left_x_bottom  = np.polyval(left_fit,  y_eval)
                right_x_bottom = np.polyval(right_fit, y_eval)
                center_x = (left_x_bottom + right_x_bottom) / 2.0

                # pixel offset from image center (positive = right of lane center)
                offset = (center_x - bw / 2.0) / (bw / 2.0)

                # heading error: difference of lane slopes at bottom
                left_slope  = 2 * left_fit[0]  * y_eval + left_fit[1]
                right_slope = 2 * right_fit[0] * y_eval + right_fit[1]
                heading_err = float((left_slope + right_slope) / 2.0)

                valid = True
            except np.linalg.LinAlgError:
                pass

        # [offset, heading_err, valid(1.0/0.0)]
        out.data = [float(offset), float(heading_err), 1.0 if valid else 0.0]
        self.pub.publish(out)

        if not valid:
            self.get_logger().warn('Lane not detected', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
