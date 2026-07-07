import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
import cv2
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

# Map YOLO class index → label used in this project
CLASS_NAMES = {
    0: 'red',
    1: 'green',
    2: 'left_arrow',
    3: 'stop_sign',
    4: 'cone',
}

DEFAULT_MODEL = 'best.pt'    # place trained weights next to the node or give full path
CONFIDENCE    = 0.5


class SignDetector(Node):
    def __init__(self):
        super().__init__('sign_detector')

        self.declare_parameter('model_path',  DEFAULT_MODEL)
        self.declare_parameter('confidence',  CONFIDENCE)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf  = self.get_parameter('confidence').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.model  = None

        if YOLO_AVAILABLE:
            try:
                self.model = YOLO(model_path)
                self.get_logger().info(f'YOLO model loaded: {model_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to load YOLO model: {e}')
        else:
            self.get_logger().error('ultralytics not installed — sign_detector will publish empty detections')

        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Detection2DArray, '/perception/sign', 10)

        self.get_logger().info('SignDetector started')

    def image_callback(self, msg: Image):
        array_msg = Detection2DArray()
        array_msg.header = msg.header

        if self.model is None:
            self.pub.publish(array_msg)
            return

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        results = self.model(img_rgb, conf=self.conf, verbose=False)

        for result in results:
            for box in result.boxes:
                cls_id    = int(box.cls[0])
                score     = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()

                det = Detection2D()
                det.header = msg.header

                det.bbox.center.position.x = (x1 + x2) / 2.0
                det.bbox.center.position.y = (y1 + y2) / 2.0
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = CLASS_NAMES.get(cls_id, str(cls_id))
                hyp.hypothesis.score    = score
                det.results.append(hyp)

                array_msg.detections.append(det)

        self.pub.publish(array_msg)

        if array_msg.detections:
            labels = [d.results[0].hypothesis.class_id for d in array_msg.detections]
            self.get_logger().info(f'Detected: {labels}')


def main(args=None):
    rclpy.init(args=args)
    node = SignDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
