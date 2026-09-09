#!/usr/bin/env python3
# =====================================================================
# 스튜디오와 ROS 사이의 유일한 창구.
#
# 규칙 하나만 지키면 됩니다: 콜백은 executor 스레드에서 도는데 Qt 위젯은 GUI
# 스레드에서만 만질 수 있으므로, 콜백은 Signal.emit만 하고 끝냅니다. 넘기는 것도
# 평범한 파이썬 값(tuple, dict, str)이지 ROS 메시지가 아닙니다.
# (waypoint_record_gui.py / hyper_rqt panel_widget.py와 같은 구조입니다.
#  executor 스레드에서 QTimer.singleShot을 부르면 조용히 무시됩니다.)
#
# ROS 그래프가 아예 없어도 창은 떠야 하므로, rclpy를 못 올리면 NullRosLink가
# 대신 들어갑니다 -- 같은 API에 available=False이고 시그널은 영영 안 웁니다.
# =====================================================================

import threading

from python_qt_binding.QtCore import QObject, Signal

# 서비스 future가 이만큼 지나도 안 오면 포기합니다(hyper_rqt의 CALL_TIMEOUT_SEC).
CALL_TIMEOUT_SEC = 5.0


def parse_status(text):
    """`key=value` 공백 구분 한 줄 -> dict.

    값에 공백이 없다는 전제입니다(파일 경로 포함 -- 경로에 공백이 있으면 여기서
    깨집니다. 그 경우 output_csv를 공백 없는 경로로 두세요).
    """
    out = {}
    for token in text.split(' '):
        key, sep, value = token.partition('=')
        if sep:
            out[key] = value
    return out


def as_float(status, key, default=None):
    try:
        return float(status[key])
    except (KeyError, ValueError, TypeError):
        return default


class NullRosLink(QObject):
    """rclpy가 없을 때 들어가는 대역. 시그널은 있고, 영영 울지 않습니다."""

    recorder_status = Signal(dict)
    recorder_path = Signal(object)
    mission_status = Signal(str)
    mission_path = Signal(object)
    mission_steps = Signal(object)
    vehicle_pose = Signal(float, float, float)
    call_finished = Signal(str, bool, str)

    available = False
    reason = 'ROS를 초기화하지 못했습니다'

    def __init__(self, reason=None):
        super().__init__()
        if reason:
            self.reason = reason

    def start(self):
        pass

    def service_ready(self, _name):
        return False

    def call(self, _name):
        self.call_finished.emit(_name, False, self.reason)

    def set_parameters(self, _node, _values, done=None):
        if done:
            done(False, self.reason)

    def shutdown(self):
        pass


class RosLink(QObject):
    """rclpy 노드 하나 + executor 스레드 하나."""

    recorder_status = Signal(dict)
    recorder_path = Signal(object)          # [(x, y)]
    mission_status = Signal(str)
    mission_path = Signal(object)           # [(x, y)]
    mission_steps = Signal(object)          # [(index, type, label, course, route)]
    vehicle_pose = Signal(float, float, float)
    call_finished = Signal(str, bool, str)  # 서비스 이름, 성공, 메시지

    available = True
    reason = ''

    def __init__(self, recorder='/waypoint_recorder', manager='/mission_manager',
                 teleport='/teleport_service', pose_topic='/odometry/filtered_map'):
        super().__init__()
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
        from nav_msgs.msg import Odometry, Path
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from rcl_interfaces.srv import SetParameters

        self._rclpy = rclpy
        self._Trigger = Trigger
        self._SetParameters = SetParameters

        self.recorder_ns = recorder.rstrip('/')
        self.manager_ns = manager.rstrip('/')
        self.teleport_ns = teleport.rstrip('/')

        self._node = Node('waypoint_studio')

        # 레코더와 미션 매니저의 status/path는 transient_local입니다 -- 창을 나중에
        # 띄워도 마지막 상태를 그대로 받습니다.
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._node.create_subscription(
            String, f'{self.recorder_ns}/status',
            lambda msg: self.recorder_status.emit(parse_status(msg.data)), latched)
        self._node.create_subscription(
            Path, f'{self.recorder_ns}/path', self._on_recorder_path, latched)
        self._node.create_subscription(
            String, f'{self.manager_ns}/status',
            lambda msg: self.mission_status.emit(msg.data), latched)
        self._node.create_subscription(
            Path, f'{self.manager_ns}/path', self._on_mission_path, latched)
        self._node.create_subscription(
            String, f'{self.manager_ns}/steps', self._on_mission_steps, latched)

        # 차량 위치. 레코더의 status에도 x/y가 있지만 주행 모드에는 레코더가 안 떠
        # 있으므로 여기서 직접 구독합니다.
        self._node.create_subscription(Odometry, pose_topic, self._on_odom, 10)

        self._clients = {}
        for name in (f'{self.recorder_ns}/start', f'{self.recorder_ns}/stop',
                     f'{self.manager_ns}/start', f'{self.manager_ns}/cancel',
                     f'{self.manager_ns}/skip', f'{self.manager_ns}/restart',
                     f'{self.manager_ns}/goto_step',
                     f'{self.teleport_ns}/teleport'):
            self._clients[name] = self._node.create_client(Trigger, name)

        self._param_clients = {}
        for node_name in (self.recorder_ns, self.manager_ns, self.teleport_ns):
            self._param_clients[node_name] = self._node.create_client(
                SetParameters, f'{node_name}/set_parameters')

        self._pending = []
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = None

    def start(self):
        """executor 스레드를 띄웁니다. 시그널을 전부 연결한 뒤에 부르세요.

        생성자에서 바로 돌리지 않는 이유: status/path/steps가 transient_local이라
        구독하는 순간 마지막 값이 곧바로 날아옵니다. 그때 아직 아무도 시그널에
        연결하지 않았으면 그 값은 그냥 사라집니다 -- steps는 미션 로드 시점에 딱
        한 번만 나가므로, 놓치면 갈래(route) 스텝이 영영 안 보입니다.
        """
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------ 콜백
    def _spin(self):
        from rclpy.executors import ExternalShutdownException
        try:
            self._executor.spin()
        except (ExternalShutdownException, KeyboardInterrupt):
            pass
        except Exception:                     # noqa: BLE001 -- 창을 죽이지 않습니다
            pass

    def _on_recorder_path(self, msg):
        self.recorder_path.emit(
            [(p.pose.position.x, p.pose.position.y) for p in msg.poses])

    def _on_mission_path(self, msg):
        self.mission_path.emit(
            [(p.pose.position.x, p.pose.position.y) for p in msg.poses])

    def _on_mission_steps(self, msg):
        """mission_manager가 펼친 스텝 목록. `index|type|label|course|route` 한 줄씩.

        yaml만 보고는 route 스텝의 인덱스를 알 수 없어서(로더가 main 뒤에 덧붙입니다)
        노드가 직접 알려 주는 값을 씁니다.
        """
        steps = []
        for line in msg.data.splitlines():
            parts = line.split('|')
            if len(parts) < 5:
                continue
            try:
                index = int(parts[0])
            except ValueError:
                continue
            steps.append((index, parts[1], parts[2], parts[3], parts[4]))
        self.mission_steps.emit(steps)

    def _on_odom(self, msg):
        import math
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.vehicle_pose.emit(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    # ------------------------------------------------------------------ 서비스
    def service_ready(self, name):
        client = self._clients.get(name)
        return bool(client and client.service_is_ready())

    def call(self, name):
        client = self._clients.get(name)
        if client is None:
            self.call_finished.emit(name, False, '알 수 없는 서비스입니다')
            return
        if not client.service_is_ready():
            self.call_finished.emit(name, False, f'{name} 서비스가 아직 없습니다')
            return
        future = client.call_async(self._Trigger.Request())
        self._pending.append((future, self._node.get_clock().now()))

        def done(fut, service=name):
            try:
                response = fut.result()
            except Exception as exc:          # noqa: BLE001 -- 표시가 목적
                self.call_finished.emit(service, False, str(exc))
                return
            self.call_finished.emit(service, response.success, response.message)

        future.add_done_callback(done)

    def set_parameters(self, node_name, values, done=None):
        """다른 노드의 파라미터를 바꿉니다 -- Trigger 서비스에 인자를 주는 방법.

        values는 {이름: 값}이고 값의 파이썬 타입으로 ParameterType을 고릅니다.
        (teleport_service의 label, model_service의 이름, mission_manager의
         step_index가 전부 같은 방식입니다.)
        """
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

        client = self._param_clients.get(node_name.rstrip('/'))
        if client is None or not client.service_is_ready():
            if done:
                done(False, f'{node_name}/set_parameters 서비스가 아직 없습니다')
            return

        parameters = []
        for name, value in values.items():
            pv = ParameterValue()
            if isinstance(value, bool):
                pv.type = ParameterType.PARAMETER_BOOL
                pv.bool_value = value
            elif isinstance(value, int):
                pv.type = ParameterType.PARAMETER_INTEGER
                pv.integer_value = value
            elif isinstance(value, float):
                pv.type = ParameterType.PARAMETER_DOUBLE
                pv.double_value = value
            else:
                pv.type = ParameterType.PARAMETER_STRING
                pv.string_value = str(value)
            parameters.append(Parameter(name=name, value=pv))

        request = self._SetParameters.Request()
        request.parameters = parameters
        future = client.call_async(request)

        def finished(fut):
            try:
                results = fut.result().results
            except Exception as exc:          # noqa: BLE001 -- 표시가 목적
                if done:
                    done(False, str(exc))
                return
            bad = [r for r in results if not r.successful]
            if bad and done:
                done(False, bad[0].reason or 'rejected')
            elif done:
                done(True, '')

        future.add_done_callback(finished)

    def drop_stale_calls(self):
        """응답이 안 오는 future를 버립니다. 죽은 서버가 버튼을 물고 있지 않도록."""
        if not self._pending:
            return
        now = self._node.get_clock().now()
        keep = []
        for future, sent in self._pending:
            if future.done():
                continue
            if (now - sent).nanoseconds * 1e-9 > CALL_TIMEOUT_SEC:
                future.cancel()
                continue
            keep.append((future, sent))
        self._pending = keep

    def shutdown(self):
        try:
            self._executor.shutdown()
        except Exception:                     # noqa: BLE001
            pass
        try:
            self._node.destroy_node()
        except Exception:                     # noqa: BLE001
            pass


def create(**kwargs):
    """RosLink를 만들되, 안 되면 NullRosLink로 떨어집니다.

    보기/편집이 이 프로그램의 기본 용도이므로 ROS가 없다고 창이 안 뜨면 안 됩니다.
    """
    try:
        import rclpy
        if not rclpy.ok():
            rclpy.init()
        return RosLink(**kwargs)
    except Exception as exc:                  # noqa: BLE001 -- 이유를 화면에 씁니다
        return NullRosLink(f'ROS 없이 실행 중입니다 ({exc})')
