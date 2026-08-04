#!/usr/bin/env python3

import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


WINDOW_NAME = 'Object Detection'
VALID_INPUT_BACKENDS = {'direct_usb', 'ros_raw'}


class ObjectDetection(Node):

    # YOLO 모델의 클래스 이름을 behavior_supervisor가 사용하는 값으로 변환
    #
    # behavior_supervisor에서 사용하는 신호:
    #   red
    #   green
    #   left_arrow
    #   none
    SIGNAL_MAP = {
        # 빨간불
        'Stop': 'red',

        # 초록불
        'Go': 'green',

        # 좌회전 화살표
        'LeftTurn': 'left_arrow',

        # Yellow light
        'Yellow': 'none',
    }

    def __init__(self):
        super().__init__('object_detection')

        # -------------------- 파라미터 --------------------
        self.declare_parameter('model_path', 'best.pt')
        self.declare_parameter('confidence_threshold', 0.5)

        # 카메라 프레임 중 몇 프레임마다 한 번 추론할지
        # 판단 노드의 신호 타임아웃이 0.4초이므로 기존 5보다 빠른 2를 기본값으로 설정
        self.declare_parameter('detection_frequency', 2)

        self.declare_parameter(
            'save_dir',
            os.path.expanduser('~/traffic_dataset')
        )

        model_path = self.get_parameter('model_path').value
        self.confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )

        self.detection_frequency = max(
            1,
            int(self.get_parameter('detection_frequency').value)
        )

        self.save_dir = self.get_parameter('save_dir').value

        os.makedirs(self.save_dir, exist_ok=True)

        # direct_usb (real vehicle -- opens the Logitech C920 itself, no ROS image topic) or
        # ros_raw (Gazebo simulation, also selectable on the real car for A/B/rollback: a plain
        # sensor_msgs/Image topic already exists). Defaults to ros_raw so a bare `ros2 run`
        # never reaches for real hardware unless a launch file opts in explicitly -- see
        # hyper_launch's real.launch.py / simulation.launch.py for which value each entrypoint
        # passes.
        self.declare_parameter('input_backend', 'ros_raw')
        input_backend = self.get_parameter('input_backend').value
        if input_backend not in VALID_INPUT_BACKENDS:
            raise ValueError(
                f"object_detection: invalid input_backend '{input_backend}' -- expected one of: "
                f'{sorted(VALID_INPUT_BACKENDS)}'
            )

        # -------------------- YOLO 및 OpenCV --------------------
        self.model = YOLO(model_path)
        self.bridge = CvBridge() if input_backend == 'ros_raw' else None

        self.n_frames = 0
        self.saved_count = 0

        # 같은 메시지를 계속 로그에 출력하지 않도록 마지막 신호 저장
        self.last_published_sign = None

        self.get_logger().info(
            f'YOLO model classes: {self.model.names}'
        )

        # -------------------- ROS 통신 --------------------
        # behavior_supervisor가 구독하는 토픽
        self.traffic_light_publisher = self.create_publisher(
            String,
            '/perception/sign',
            10
        )

        self.image_subscriber = None
        self.capture = None
        self.capture_timer = None

        if input_backend == 'ros_raw':
            self.image_subscriber = self.create_subscription(
                Image,
                '/image_raw',
                self.image_callback,
                qos_profile_sensor_data
            )
        else:
            self._setup_direct_usb_capture()

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

        self.get_logger().info(
            f'ObjectDetection started (input_backend={input_backend}): '
            'publishing /perception/sign'
        )

    def _setup_direct_usb_capture(self):
        # Owns the Logitech C920 directly -- mirrors hyper_lane_detection's ElpCameraCapture for
        # the ELP camera, minus rectification: object_detection_node has always consumed the raw
        # frame as-is, and this camera has no intrinsic calibration.
        self.declare_parameter('video_device', '/dev/video_logitech')
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('framerate', 30.0)

        video_device = self.get_parameter('video_device').value
        image_width = int(self.get_parameter('image_width').value)
        image_height = int(self.get_parameter('image_height').value)
        framerate = float(self.get_parameter('framerate').value)

        self.capture = cv2.VideoCapture(video_device, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            raise RuntimeError(
                f"object_detection: direct_usb failed to open camera device '{video_device}'"
            )

        # MJPEG in, decoded to BGR8 out -- OpenCV's V4L2 backend does the JPEG decode internally
        # on read() (CAP_PROP_CONVERT_RGB stays at its default True).
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, image_width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, image_height)
        self.capture.set(cv2.CAP_PROP_FPS, framerate)

        self.get_logger().info(
            f"direct_usb: opened '{video_device}' "
            f'({image_width}x{image_height} @ {framerate:.0f} fps)'
        )

        self.capture_timer = self.create_timer(
            1.0 / framerate,
            self.capture_timer_callback
        )

    def publish_sign(self, sign_name, force_log=False):
        """
        판단 노드가 사용하는 형식으로 신호를 발행한다.

        가능한 값:
            red
            green
            left_arrow
            none
        """
        valid_signs = {
            'red',
            'green',
            'left_arrow',
            'none'
        }

        if sign_name not in valid_signs:
            sign_name = 'none'

        msg = String()
        msg.data = sign_name
        self.traffic_light_publisher.publish(msg)

        # 신호가 바뀌었을 때만 로그 출력
        if force_log or sign_name != self.last_published_sign:
            self.get_logger().info(
                f'Published sign: {sign_name}'
            )

        self.last_published_sign = sign_name

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

        except Exception as e:
            self.get_logger().error(
                f'cv_bridge exception: {e}'
            )
            return

        self.process_frame(frame)

    def capture_timer_callback(self):
        ok, frame = self.capture.read()
        if not ok or frame is None:
            # Skip this tick rather than aborting the node over what may be a transient USB
            # glitch -- same treatment as ElpCameraCapture::read() on the lane detection side.
            self.get_logger().error(
                'direct_usb: camera stream read failed (device disconnected?)'
            )
            return

        self.process_frame(frame)

    def process_frame(self, frame):
        self.n_frames += 1

        # 지정된 프레임 간격마다 YOLO 추론
        if self.n_frames % self.detection_frequency != 0:
            return

        try:
            results = self.model(
                frame,
                conf=self.confidence_threshold,
                verbose=False
            )

        except Exception as e:
            self.get_logger().error(
                f'YOLO inference exception: {e}'
            )
            self.publish_sign('none')
            return

        if results is None or len(results) == 0:
            self.publish_sign('none')
            return

        result = results[0]
        annotated_frame = result.plot()

        self.publish_middle_traffic_light(
            result,
            frame.shape[1]
        )

        cv2.imshow(WINDOW_NAME, annotated_frame)
        cv2.waitKey(1)

    def publish_middle_traffic_light(self, result, image_width):
        """
        화면 중앙 50% 영역에 있는 신호 객체 중에서 하나를 선택한다.

        선택 우선순위:
        1. 화면 중앙에 가장 가까운 객체
        2. 중앙까지 거리가 같으면 신뢰도가 높은 객체

        자동차, 사람 등 신호와 관련 없는 클래스는 제외한다.
        """
        boxes = result.boxes

        # 검출 결과가 없으면 none
        if boxes is None or len(boxes) == 0:
            self.publish_sign('none')
            return

        left_bound = image_width * 0.25
        right_bound = image_width * 0.75
        image_center = image_width / 2.0

        best_box = None
        best_key = None
        best_raw_class_name = None
        best_sign_name = None

        for box in boxes:
            class_id = int(box.cls[0])

            raw_class_name = str(
                self.model.names[class_id]
            )

            # 판단 노드에서 사용할 수 있는 신호 클래스로 변환
            sign_name = self.SIGNAL_MAP.get(
                raw_class_name
            )

            # 자동차, 사람, 표지판 등 신호가 아닌 클래스는 제외
            if sign_name is None:
                continue

            x1, _, x2, _ = box.xyxy[0].tolist()
            box_center = (x1 + x2) / 2.0

            # 화면 중앙 50% 바깥에 있는 신호는 제외
            if not (left_bound <= box_center <= right_bound):
                continue

            distance_to_center = abs(
                box_center - image_center
            )

            confidence = float(box.conf[0])

            # 중앙에 가까운 객체 우선
            # 중앙 거리가 같으면 confidence가 높은 객체 우선
            key = (
                distance_to_center,
                -confidence
            )

            if best_key is None or key < best_key:
                best_key = key
                best_box = box
                best_raw_class_name = raw_class_name
                best_sign_name = sign_name

        # 중앙 영역에서 유효한 신호를 찾지 못함
        if best_box is None:
            self.publish_sign('none')
            return

        confidence = float(best_box.conf[0])

        self.publish_sign(best_sign_name)

        self.get_logger().info(
            f'Detected traffic light: '
            f'{best_raw_class_name} -> {best_sign_name} '
            f'({confidence:.2f})'
        )

    def destroy_node(self):
        if self.capture is not None:
            self.capture.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ObjectDetection()

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
