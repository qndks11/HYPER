#!/usr/bin/env python3

import os

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class ObjectDetection(Node):

    # mission_manager가 쓰는 신호 값의 전체 목록.
    #
    # 신호등: red / green / left_arrow
    # 차선 안내: ban / allow  -- 코스 끝의 갈림길에서 어느 차선으로 갈지 알려 주는 표지.
    #            mission.yaml의 branch 스텝이 이 값을 보고 두 갈래 중 하나를 고릅니다.
    # none:    유효한 신호를 못 봤다.
    VALID_SIGNS = frozenset({
        'red',
        'green',
        'left_arrow',
        'ban',
        'allow',
        'none',
    })

    # YOLO 모델의 클래스 이름 -> 위 신호 값.
    #
    # 여기 없는 클래스는 "신호가 아닌 것"(자동차, 사람 등)으로 보고 무시합니다. 학습 모델의
    # 클래스 이름이 바뀌면 신호가 조용히 사라지므로, 무시한 이름은 한 번씩 경고로 남깁니다
    # (아래 _map_class). 코드를 안 고치고 맞추려면 sign_class_map 파라미터를 쓰세요.
    SIGNAL_MAP = {
        # 빨간불
        'Stop': 'red',

        # 초록불
        'Go': 'green',

        # 좌회전 화살표
        'LeftTurn': 'left_arrow',

        # Yellow light
        'Yellow': 'none',

        # 차선 안내 표지 -- 실제 학습 모델의 클래스 이름에 맞춰 넣은 후보들입니다.
        # 모델이 다른 이름을 쓰면 sign_class_map 파라미터로 덮어쓰세요.
        'Ban': 'ban',
        'NoEntry': 'ban',
        'Allow': 'allow',
        'Entry': 'allow',
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

        # YOLO 클래스 이름 -> 신호 값 추가/덮어쓰기. "YoloClass:sign" 형태의 문자열 목록입니다.
        # 예: ros2 run ... --ros-args -p sign_class_map:="['LaneBan:ban','LaneAllow:allow']"
        # 학습 모델의 클래스 이름은 팀마다/버전마다 달라지는데, 그때마다 이 파일을 고치는 대신
        # 런치에서 맞출 수 있게 한 것입니다.
        self.declare_parameter('sign_class_map', [''])

        model_path = self.get_parameter('model_path').value
        self.confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )

        self.detection_frequency = max(
            1,
            int(self.get_parameter('detection_frequency').value)
        )

        self.save_dir = self.get_parameter('save_dir').value

        self.signal_map = dict(self.SIGNAL_MAP)
        for entry in self.get_parameter('sign_class_map').value or []:
            if not entry.strip():
                continue
            raw_name, _, sign_name = entry.partition(':')
            raw_name, sign_name = raw_name.strip(), sign_name.strip()
            if not raw_name or sign_name not in self.VALID_SIGNS:
                self.get_logger().error(
                    f"sign_class_map entry '{entry}' is not '<YoloClass>:<sign>' with sign in "
                    f'{sorted(self.VALID_SIGNS)}; ignoring it.'
                )
                continue
            self.signal_map[raw_name] = sign_name

        # 무시한 클래스 이름은 한 번씩만 경고합니다(매 프레임 찍으면 로그가 묻힙니다).
        self.unmapped_classes = set()

        os.makedirs(self.save_dir, exist_ok=True)

        # -------------------- YOLO 및 OpenCV --------------------
        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        self.n_frames = 0
        self.saved_count = 0

        # 같은 메시지를 계속 로그에 출력하지 않도록 마지막 신호 저장
        self.last_published_sign = None

        self.get_logger().info(
            f'YOLO model classes: {self.model.names}'
        )
        self.get_logger().info(
            f'Sign class map: {self.signal_map}'
        )

        # -------------------- ROS 통신 --------------------
        # behavior_supervisor가 구독하는 토픽
        self.traffic_light_publisher = self.create_publisher(
            String,
            '/perception/sign',
            10
        )

        # /image_raw is fed by ros_gz_bridge in Gazebo sim, or by hyper_camera's
        # logitech_camera_publisher_node on the real vehicle -- see hyper_object_detection's
        # perception.launch.py for the remap/node selection either way. rclpy has no
        # use_intra_process_comms equivalent to rclcpp's, so unlike hyper_lane_detection this is
        # always a plain topic subscription, never a same-process zero-copy path.
        self.image_subscriber = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        # YOLO 검출 결과를 그린 프레임을 rviz에서 볼 수 있도록 발행 (imshow 대체)
        self.annotated_image_publisher = self.create_publisher(
            Image,
            '/perception/object_detection/annotated_image',
            qos_profile_sensor_data
        )

        self.get_logger().info(
            'ObjectDetection started: publishing /perception/sign'
        )

    def publish_sign(self, sign_name, force_log=False):
        """
        판단 노드가 사용하는 형식으로 신호를 발행한다.

        가능한 값은 VALID_SIGNS 참고 (red / green / left_arrow / ban / allow / none).
        """
        if sign_name not in self.VALID_SIGNS:
            # 여기로 오면 SIGNAL_MAP이나 sign_class_map이 VALID_SIGNS에 없는 값을 냈다는
            # 뜻입니다. 'none'으로 바꿔 내보내되(안전한 쪽), 조용히 넘기지는 않습니다 --
            # mission_manager 쪽에서는 "신호가 영영 안 온다"로만 보여 원인을 찾기 어렵습니다.
            self.get_logger().error(
                f"Sign '{sign_name}' is not one of {sorted(self.VALID_SIGNS)}; "
                'publishing none instead.'
            )
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

        self.process_frame(frame, msg.header)

    def process_frame(self, frame, header):
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

        self.publish_center_sign(
            result,
            frame.shape[1]
        )

        annotated_msg = self.bridge.cv2_to_imgmsg(
            annotated_frame,
            encoding='bgr8'
        )
        annotated_msg.header = header
        self.annotated_image_publisher.publish(annotated_msg)

    def _map_class(self, raw_class_name):
        """
        YOLO 클래스 이름을 신호 값으로 바꾼다. 신호가 아니면 None.

        매핑에 없는 이름은 한 번씩 경고로 남긴다. 학습 모델의 클래스 이름이 바뀌면
        (예: 'Ban' -> 'LaneBan') 신호가 조용히 사라지는데, 그 증상은 mission_manager
        쪽에서 "신호가 영영 안 온다"로만 보여 원인을 찾기 어렵기 때문이다.
        """
        sign_name = self.signal_map.get(raw_class_name)
        if sign_name is None and raw_class_name not in self.unmapped_classes:
            self.unmapped_classes.add(raw_class_name)
            self.get_logger().warn(
                f"YOLO class '{raw_class_name}' is not in the sign class map, so it is "
                'ignored. If this is a traffic light or a lane-guidance sign, add it with '
                "the sign_class_map parameter (e.g. -p sign_class_map:=\"['"
                f'{raw_class_name}:ban\']").'
            )
        return sign_name

    def publish_center_sign(self, result, image_width):
        """
        화면 중앙 50% 영역에 있는 신호 객체 중에서 하나를 선택한다.

        신호등과 차선 안내 표지가 같은 프레임에 보이면 중앙에 가까운 쪽 하나만 나간다.
        그래도 안전한데, mission_manager의 판정은 "같은 값이 연속 N프레임"이라 두 표지가
        번갈아 나오면 어느 쪽도 확정되지 않기 때문이다 -- 신호등 앞에서는 계속 서 있고,
        갈림길에서는 timeout 뒤 default 갈래로 간다. 둘 다 안전한 쪽 실패다.

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

            # 판단 노드에서 사용할 수 있는 신호 클래스로 변환.
            # 자동차, 사람 등 신호가 아닌 클래스는 여기서 걸러집니다.
            sign_name = self._map_class(raw_class_name)
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
