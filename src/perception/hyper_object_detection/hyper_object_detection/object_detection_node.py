#!/usr/bin/env python3

import os
from datetime import datetime

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool
from ultralytics import YOLO


class ObjectDetection(Node):

    # mission_manager가 쓰는 신호 값의 전체 목록.
    #
    # 신호등: red / green / left_arrow
    # 차선 안내: ban / allow  -- 코스 끝의 갈림길에서 어느 차선으로 갈지 알려 주는 표지.
    #            mission.yaml의 branch 스텝이 이 값을 보고 두 갈래 중 하나를 고릅니다.
    # 차선 안내(상대 위치): allow_left / allow_right  -- 'allow'와 'ban' 표지가 한 프레임에
    #            같이 보일 때, 'allow'가 'ban'보다 화면에서 왼쪽에 있으면 allow_left,
    #            오른쪽에 있으면 allow_right. "허용 차선으로 가라"를 어느 쪽 차선인지까지
    #            알려 주는 갈림길에서 branch 스텝이 이 값을 봅니다.
    # none:    유효한 신호를 못 봤다.
    VALID_SIGNS = frozenset({
        'red',
        'green',
        'left_arrow',
        'ban',
        'allow',
        'allow_left',
        'allow_right',
        'none',
    })

    # YOLO 모델의 클래스 이름 -> 위 신호 값.
    #
    # models/best.pt가 실제로 가진 클래스는 여섯입니다:
    #   Allow, Ban, Go, LeftTurn, Stop, Warn
    # 아래 매핑은 그 여섯을 전부 덮습니다. 모델을 다시 학습해 이름이 바뀌면 그 신호가
    # 조용히 사라지므로, 매핑에 없는 이름은 한 번씩 경고로 남깁니다(아래 _map_class).
    # 코드를 안 고치고 맞추려면 sign_class_map 파라미터를 쓰세요.
    SIGNAL_MAP = {
        # 빨간불
        'Stop': 'red',

        # 초록불
        'Go': 'green',

        # 좌회전 화살표
        'LeftTurn': 'left_arrow',

        # 황색등. 'none'으로 두는 것은 "무시"가 아니라 "통과 신호가 아니다"입니다 --
        # 매핑에서 빼면 이 박스가 중앙 선택에서 아예 제외되어, 화면 가장자리의 다른
        # 표지가 대신 뽑힐 수 있습니다. 'none'이면 황색등이 중앙을 차지한 채 통과
        # 신호가 아님을 알리므로, wait_signal의 연속 프레임이 거기서 끊깁니다.
        # ('Yellow'는 예전 모델의 이름입니다. 지금 모델은 'Warn'을 씁니다.)
        'Warn': 'none',
        'Yellow': 'none',

        # 차선 안내 표지 -- 코스 끝 갈림길에서 branch 스텝이 봅니다.
        'Ban': 'ban',
        'Allow': 'allow',
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

        # `~/image_saving` 서비스로 켜는 데이터 수집 모드가 쓰는 값들입니다.
        # 상대 경로면 런치를 실행한 작업 디렉터리 기준으로 풀립니다.
        # 저장 여부 자체는 파라미터가 아닙니다 -- 오래된 설정 파일 때문에 노드가
        # 저 혼자 녹화를 시작하는 일이 없도록, 오직 서비스 호출로만 켜집니다.
        self.declare_parameter('image_save_dir', 'data/object_detection')

        # 카메라 프레임 사이 간격이 워낙 좁아 연속 프레임은 거의 같은 그림입니다.
        # 같은 용량으로 더 다양한 장면을 담으려고 일부러 낮게 잡은 기본값입니다.
        self.declare_parameter('image_save_rate', 2.0)

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

        # -------------------- 이미지 저장 모드 --------------------
        self.image_saving_enabled = False
        self.image_save_dir = self.get_parameter('image_save_dir').value
        image_save_rate = float(self.get_parameter('image_save_rate').value)
        if image_save_rate <= 0.0:
            self.get_logger().warn(
                f'image_save_rate must be > 0 (got {image_save_rate}); using 2.0 Hz'
            )
            image_save_rate = 2.0
        self.image_save_period = 1.0 / image_save_rate
        self.last_image_save_time = None

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

        # /image_raw is remapped to /camera/image_raw -- the vehicle's one camera, shared with
        # lane_detection. Fed by ros_gz_bridge in Gazebo sim, or on the real vehicle by
        # hyper_camera's LogitechCameraPublisherNode component, which lives inside
        # lane_detection's container but publishes over DDS all the same. See
        # hyper_object_detection's perception.launch.py. rclpy has no use_intra_process_comms
        # equivalent to rclcpp's, so unlike hyper_lane_detection this is always a plain topic
        # subscription, never a same-process zero-copy path.
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

        # 데이터 수집 모드 토글. 파라미터가 아니라 서비스인 이유는, 켠 뒤에
        # 운영자가 정작 알아야 하는 "어디에 쌓이는지"를 응답으로 돌려주기 때문입니다.
        self.image_saving_service = self.create_service(
            SetBool,
            '~/image_saving',
            self.handle_image_saving
        )

        self.get_logger().info(
            'ObjectDetection started: publishing /perception/sign'
        )

    def handle_image_saving(self, request, response):
        """
        std_srvs/SetBool: 이미지 저장 모드를 켜거나(data: true) 끈다(data: false).

        같은 상태로 다시 불러도 문제 없이 경로만 다시 알려주므로, 런치 스크립트가
        직전 실행이 뭘 남겼든 신경 쓰지 않고 "꺼짐"을 확정할 수 있습니다.
        폴더는 노드 시작 때가 아니라 여기서 만듭니다 -- 한 번도 녹화하지 않은
        노드가 빈 폴더를 남기지 않도록.
        """
        absolute = os.path.abspath(self.image_save_dir)

        if not request.data:
            self.image_saving_enabled = False
            response.success = True
            response.message = f'image saving off ({absolute})'
            self.get_logger().info('image saving off')
            return response

        try:
            os.makedirs(absolute, exist_ok=True)
        except OSError as exc:
            # 실패를 로그로만 남기면, 녹화를 켠 줄 알고 주행을 마친 운영자가
            # 빈손으로 돌아옵니다. 서비스 실패로 돌려줍니다.
            self.image_saving_enabled = False
            response.success = False
            response.message = f'cannot create {absolute}: {exc}'
            self.get_logger().error(response.message)
            return response

        # 켠 직후 첫 프레임은 한 주기 기다리지 않고 바로 저장되도록 시계를 비웁니다.
        self.last_image_save_time = None
        self.image_saving_enabled = True
        response.success = True
        response.message = (
            f'image saving on at {1.0 / self.image_save_period:.1f} fps -> {absolute}'
        )
        self.get_logger().info(response.message)
        return response

    def save_frame_if_due(self, frame):
        """
        저장 모드가 켜져 있고 마지막 저장 이후 image_save_period 만큼 지났으면 한 장 쓴다.

        간격은 노드 시계(use_sim_time이면 시뮬레이션 시계) 기준입니다. 그래야 배속을
        올리거나 내린 시뮬에서도 차량이 "본" 시간에 비례해 기록이 남습니다.
        카메라가 저장 속도보다 훨씬 빠르므로 대부분의 프레임은 의도적으로 버립니다.

        저장에 실패해도 예외를 올리지 않습니다 -- 디스크가 꽉 찼다고 해서, 녹화가
        곁다리로 붙어 있는 검출 파이프라인까지 멈출 이유는 없습니다.
        """
        if not self.image_saving_enabled:
            return

        now = self.get_clock().now()
        if self.last_image_save_time is not None:
            # 음수(시뮬 리셋 등으로 시계가 뒤로 뛴 경우)도 저장하고 시계를 다시 잡습니다.
            elapsed = (now - self.last_image_save_time).nanoseconds / 1e9
            if 0.0 <= elapsed < self.image_save_period:
                return
        self.last_image_save_time = now

        # 파일 이름은 벽시계 기준입니다. 사람이 실제 주행과 맞춰 보는 용도인 데다,
        # 0에서 시작하는 시뮬 시계로 이름을 지으면 실행마다 이름이 겹칩니다.
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        path = os.path.join(
            os.path.abspath(self.image_save_dir),
            f'objdet_{stamp}.png'
        )

        try:
            if not cv2.imwrite(path, frame):
                raise OSError('cv2.imwrite returned False')
        except (OSError, cv2.error) as exc:
            self.get_logger().error(
                f'failed to write {path}: {exc}',
                throttle_duration_sec=5.0
            )
            return

        self.saved_count += 1

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

        # 추론 주기와 무관하게 카메라 프레임마다 후보로 들어옵니다. 저장 간격은
        # save_frame_if_due가 시간으로 직접 재므로, detection_frequency를 바꿔도
        # 기록 속도는 흔들리지 않습니다.
        self.save_frame_if_due(frame)

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

        # 'allow'와 'ban'이 한 프레임에 같이 보이면 둘의 좌우 배치가 곧 신호이므로
        # (allow_left / allow_right) 중앙 선택보다 먼저 본다. 둘 중 하나만 보이거나
        # 아예 없으면 False를 돌려주고 평소의 중앙 선택으로 넘어간다.
        if not self.publish_lane_fork_sign(result):
            self.publish_center_sign(
                result,
                frame.shape[1]
            )

        self.publish_annotated_image(result, header)

    def publish_annotated_image(self, result, header):
        """
        검출 박스를 그린 프레임을 발행한다. 보는 사람이 없으면 그리지 않는다.

        result.plot()은 프레임을 복사해 박스를 전부 그리고, cv2_to_imgmsg가 그 결과를
        한 번 더 복사합니다. 둘 다 추론 한 번마다 드는 비용인데, rviz나 rqt를 안 띄운
        실주행에서는 아무도 안 보는 그림에 그 비용을 씁니다. 구독자가 생기면 다음
        프레임부터 다시 그리므로, 주행 중에 rviz를 켜도 그대로 보입니다.
        (hyper_lane_detection의 디버그 이미지/포인트클라우드도 같은 방식입니다.)
        """
        if self.annotated_image_publisher.get_subscription_count() == 0:
            return

        annotated_msg = self.bridge.cv2_to_imgmsg(
            result.plot(),
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

    def publish_lane_fork_sign(self, result):
        """
        'allow'와 'ban' 표지가 한 프레임에 같이 보이면 좌우 배치를 신호로 낸다.

        allow가 ban보다 왼쪽이면 'allow_left', 오른쪽이면 'allow_right'를 publish하고
        True를 돌려준다. 둘 중 하나만 보이거나 아예 없으면 아무것도 안 하고 False --
        그러면 호출부가 평소의 publish_center_sign으로 넘어간다.

        중앙 50% 제한을 두지 않는다: 갈림길 표지는 보통 나란히 붙어 있어 한쪽이 중앙
        밖으로 밀리기 쉽고, 여기서 중요한 건 화면 어디에 있느냐가 아니라 둘의 상대
        위치이기 때문이다. 같은 클래스가 여러 개면 신뢰도가 가장 높은 박스를 쓴다.
        """
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return False

        best = {'allow': None, 'ban': None}  # sign -> (confidence, box_center_x)
        for box in boxes:
            raw_class_name = str(self.model.names[int(box.cls[0])])
            sign_name = self._map_class(raw_class_name)
            if sign_name not in ('allow', 'ban'):
                continue

            confidence = float(box.conf[0])
            x1, _, x2, _ = box.xyxy[0].tolist()
            box_center = (x1 + x2) / 2.0
            if best[sign_name] is None or confidence > best[sign_name][0]:
                best[sign_name] = (confidence, box_center)

        if best['allow'] is None or best['ban'] is None:
            return False

        allow_x = best['allow'][1]
        ban_x = best['ban'][1]
        self.publish_sign('allow_left' if allow_x < ban_x else 'allow_right')
        return True

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
