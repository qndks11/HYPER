#!/usr/bin/env python3
"""config/panel.yaml에 적힌 서비스만 버튼으로 보여 주는 rqt 위젯.

왜 rqt_service_caller를 안 쓰는가: 그쪽 드롭다운에는 실행 중인 *모든* 서비스가
나옵니다. 노드 하나마다 파라미터 서비스가 6개씩 붙고 Gazebo/RViz 내부 서비스까지
합치면 100개가 넘는데, 사람이 실제로 부르는 것은 mission_manager 4개와
model_service 2개뿐입니다.

스레드: 노드는 rqt가 별도 QThread의 MultiThreadedExecutor로 spin 합니다
(rqt_gui_py/rclpy_spinner.py). 그래서 서비스 응답 콜백과 토픽 콜백은 GUI 스레드가
아닌 곳에서 불립니다 -- 위젯을 직접 건드리면 안 되고, Signal로 넘겨야 합니다.
아래 _call_finished / _status_received가 그 경계입니다.
"""
import os
import time

import yaml
from python_qt_binding.QtCore import Qt, QTimer, Signal
from python_qt_binding.QtWidgets import (
    QComboBox, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QMessageBox, QPlainTextEdit, QPushButton, QSizePolicy, QVBoxLayout, QWidget,
)
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_service
from std_msgs.msg import String

DEFAULT_SERVICE_TYPE = 'std_srvs/srv/Trigger'
# 응답이 이 시간 안에 안 오면 버튼을 다시 살립니다. 서버가 죽으면 future는 영영
# 완료되지 않아서, 이게 없으면 버튼이 비활성으로 굳습니다.
CALL_TIMEOUT_SEC = 5.0
# 서비스가 떠 있는지 확인해 버튼을 켜고 끄는 주기.
POLL_PERIOD_MS = 1000

_BUTTON_STYLES = {
    'go': ('#1b7f3b', '#ffffff'),
    'stop': ('#a32424', '#ffffff'),
    'warn': ('#a86a00', '#ffffff'),
}


def _style_sheet(style):
    colors = _BUTTON_STYLES.get(style)
    if colors is None:
        return ''
    background, foreground = colors
    return (
        'QPushButton {{ background-color: {bg}; color: {fg}; font-weight: bold;'
        ' border: none; border-radius: 3px; padding: 8px 14px; }}'
        'QPushButton:hover:enabled {{ background-color: {bg}; opacity: 0.8; }}'
        'QPushButton:disabled {{ background-color: #6b6b6b; color: #d0d0d0; }}'
    ).format(bg=background, fg=foreground)


def _coerce(value):
    """YAML 값을 rclpy Parameter가 받는 형태로 맞춥니다.

    리스트 안에 int와 float이 섞여 있으면 Parameter가 타입을 못 정하고 예외를
    냅니다. pose: [-37.77, 3.72, 0.0, ...] 처럼 하나만 정수로 적어도 터지므로,
    숫자 리스트는 전부 float으로 올려 둡니다.
    """
    if isinstance(value, (list, tuple)) and value:
        if all(isinstance(v, bool) for v in value):
            return list(value)
        if all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in value):
            if any(isinstance(v, float) for v in value):
                return [float(v) for v in value]
        return list(value)
    return value


class ButtonSpec(object):
    """panel.yaml의 buttons[] 한 항목."""

    def __init__(self, group, index, raw):
        self.key = '{}/{}'.format(group, index)
        self.label = raw.get('label', raw.get('service', '?'))
        self.service = raw['service']
        self.type_name = raw.get('type', DEFAULT_SERVICE_TYPE)
        self.request = raw.get('request') or {}
        self.style = raw.get('style', '')
        self.confirm = bool(raw.get('confirm', False))
        self.tooltip = raw.get('tooltip', '')


class SelectSpec(object):
    """panel.yaml의 selects[] 한 항목 -- 대상 노드의 파라미터 프리셋."""

    def __init__(self, raw):
        self.label = raw.get('label', '선택')
        self.node = raw['node']
        self.apply_on_start = bool(raw.get('apply_on_start', False))
        self.default = raw.get('default')
        self.options = []
        param = raw.get('param')
        for option in raw.get('options') or []:
            if isinstance(option, dict):
                self.options.append((option.get('label', '?'), option.get('params') or {}))
            elif param:
                self.options.append((str(option), {param: option}))
            else:
                raise ValueError(
                    "select '{}': options에 스칼라를 쓰려면 param을 함께 적어야 "
                    '합니다'.format(self.label))


class HyperPanelWidget(QWidget):

    _call_finished = Signal(str, bool, str)
    _status_received = Signal(str)

    def __init__(self, node, config_path):
        super(HyperPanelWidget, self).__init__()
        self._node = node
        self._config_path = config_path
        self._clients = {}
        self._param_clients = {}
        self._pending = {}
        self._pending_selects = {}
        self._buttons = {}
        self._status_sub = None

        self.setObjectName('HyperPanelWidget')
        self.setWindowTitle('HYPER Panel')

        self._call_finished.connect(self._on_call_finished)
        self._status_received.connect(self._on_status_received)

        with open(config_path) as handle:
            config = yaml.safe_load(handle) or {}

        self._build_ui(config)
        self._subscribe_status(config.get('status_topic'))

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._poll)
        self._timer.start(POLL_PERIOD_MS)

    # ------------------------------------------------------------------ UI

    def _build_ui(self, config):
        layout = QVBoxLayout(self)

        # 로그 위젯을 먼저 만듭니다 -- 아래 _build_group()이 apply_on_start를
        # 처리하면서 이미 로그를 씁니다.
        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(200)
        self._log.setMinimumHeight(90)

        self._status_label = QLabel('-')
        self._status_label.setAlignment(Qt.AlignCenter)
        self._status_label.setStyleSheet(
            'font-size: 15pt; font-weight: bold; padding: 10px;'
            ' border-radius: 4px; background-color: #3a3a3a; color: #dddddd;')
        self._status_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(self._status_label)

        for group in config.get('groups') or []:
            layout.addWidget(self._build_group(group))

        layout.addWidget(self._log, 1)

        footer = QLabel(os.path.basename(self._config_path) + ' -- ' + self._config_path)
        footer.setStyleSheet('color: #888888; font-size: 8pt;')
        footer.setToolTip('버튼을 추가하려면 이 파일을 고치세요.')
        layout.addWidget(footer)

    def _build_group(self, group):
        name = group.get('name', '')
        box = QGroupBox(name)
        box_layout = QVBoxLayout(box)

        selects = group.get('selects') or []
        if selects:
            form = QFormLayout()
            form.setContentsMargins(0, 0, 0, 6)
            for raw in selects:
                spec = SelectSpec(raw)
                combo = QComboBox()
                for label, params in spec.options:
                    combo.addItem(label, params)
                if spec.default is not None:
                    index = combo.findText(str(spec.default))
                    if index >= 0:
                        combo.setCurrentIndex(index)
                combo.currentIndexChanged.connect(
                    lambda _index, c=combo, s=spec: self._apply_select(s, c))
                form.addRow(spec.label, combo)
                if spec.apply_on_start and combo.count():
                    self._apply_select(spec, combo)
            box_layout.addLayout(form)

        grid = QGridLayout()
        for index, raw in enumerate(group.get('buttons') or []):
            spec = ButtonSpec(name, index, raw)
            button = QPushButton(spec.label)
            button.setStyleSheet(_style_sheet(spec.style))
            button.setMinimumHeight(38)
            button.setToolTip(
                '{}\n{}\n{}'.format(spec.tooltip, spec.service, spec.type_name).strip())
            button.clicked.connect(lambda _checked=False, s=spec: self._on_click(s))
            grid.addWidget(button, index // 4, index % 4)
            self._buttons[spec.key] = (button, spec)
        box_layout.addLayout(grid)
        return box

    # ------------------------------------------------------------- 서비스 호출

    def _client(self, service, type_name):
        key = (service, type_name)
        if key not in self._clients:
            self._clients[key] = self._node.create_client(get_service(type_name), service)
        return self._clients[key]

    def _on_click(self, spec):
        if spec.confirm:
            answer = QMessageBox.question(
                self, spec.label,
                '{} 을(를) 호출합니다.\n\n{}'.format(spec.label, spec.service),
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                return

        client = self._client(spec.service, spec.type_name)
        if not client.service_is_ready():
            self._append_log(spec.label, False, '서비스가 없습니다: ' + spec.service)
            return

        request = get_service(spec.type_name).Request()
        if spec.request:
            try:
                set_message_fields(request, spec.request)
            except Exception as exc:  # noqa: BLE001 -- YAML 오타를 로그로 보여 줍니다
                self._append_log(spec.label, False, 'request 채우기 실패: {}'.format(exc))
                return

        button = self._buttons[spec.key][0]
        button.setEnabled(False)
        future = client.call_async(request)
        self._pending[spec.key] = (future, time.monotonic())
        # 아래 콜백은 executor 스레드에서 불립니다 -- Signal로만 GUI에 닿습니다.
        future.add_done_callback(
            lambda fut, key=spec.key, label=spec.label: self._call_done(key, label, fut))

    def _call_done(self, key, label, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self._call_finished.emit(label, False, str(exc))
            return
        if response is None:
            self._call_finished.emit(label, False, '응답 없음 (취소됨)')
            return
        success = getattr(response, 'success', True)
        message = getattr(response, 'message', None)
        if message is None:
            message = str(response)
        self._call_finished.emit(label, bool(success), str(message))
        self._pending.pop(key, None)

    def _on_call_finished(self, label, success, message):
        self._append_log(label, success, message)
        for key, (button, spec) in self._buttons.items():
            if spec.label == label:
                button.setEnabled(True)
        self._pending = {
            key: value for key, value in self._pending.items() if not value[0].done()}

    # ------------------------------------------------------------ 파라미터 적용

    def _apply_select(self, spec, combo):
        params = combo.currentData()
        if not params:
            return
        if spec.node not in self._param_clients:
            self._param_clients[spec.node] = self._node.create_client(
                SetParameters, spec.node.rstrip('/') + '/set_parameters')
        client = self._param_clients[spec.node]
        if not client.service_is_ready():
            # 패널을 스택보다 먼저 띄우는 게 보통이라, 여기서 포기하면 프리셋이
            # 영영 적용되지 않습니다. 노드가 뜰 때까지 _poll()이 다시 시도합니다.
            key = (spec.node, spec.label)
            if key not in self._pending_selects:
                self._pending_selects[key] = (spec, combo)
                self._append_log(
                    spec.label, False,
                    '{} 가 아직 없습니다 -- 뜨면 적용합니다'.format(spec.node))
            return
        self._pending_selects.pop((spec.node, spec.label), None)
        request = SetParameters.Request()
        request.parameters = [
            Parameter(name=name, value=_coerce(value)).to_parameter_msg()
            for name, value in params.items()]
        future = client.call_async(request)
        future.add_done_callback(
            lambda fut, label=spec.label, node=spec.node, keys=list(params):
                self._param_done(label, node, keys, fut))

    def _param_done(self, label, node, keys, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self._call_finished.emit(label, False, str(exc))
            return
        failed = [
            keys[i] for i, result in enumerate(response.results) if not result.successful]
        if failed:
            self._call_finished.emit(label, False, '{} 설정 실패: {}'.format(node, failed))
        else:
            self._call_finished.emit(label, True, '{} <- {}'.format(node, ', '.join(keys)))

    # ------------------------------------------------------------------ 상태

    def _subscribe_status(self, topic):
        if not topic:
            return
        # mission_manager는 status를 transient_local로 latch 합니다. 나중에 붙는
        # 이 패널도 마지막 상태를 바로 받으려면 같은 durability여야 합니다.
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_sub = self._node.create_subscription(
            String, topic, lambda msg: self._status_received.emit(msg.data), qos)

    def _on_status_received(self, text):
        lowered = text.lower()
        if 'fail' in lowered or 'error' in lowered:
            background = '#a32424'
        elif 'blocked' in lowered or 'wait' in lowered:
            background = '#a86a00'
        elif lowered.startswith('idle') or 'cancel' in lowered:
            background = '#3a3a3a'
        elif 'finished' in lowered:
            background = '#1f5c8b'
        else:
            background = '#1b7f3b'
        self._status_label.setText(text)
        self._status_label.setStyleSheet(
            'font-size: 15pt; font-weight: bold; padding: 10px; border-radius: 4px;'
            ' background-color: {}; color: #ffffff;'.format(background))

    # ------------------------------------------------------- 주기 확인 / 로그

    def _poll(self):
        for spec, combo in list(self._pending_selects.values()):
            self._apply_select(spec, combo)

        now = time.monotonic()
        for key, (future, started) in list(self._pending.items()):
            if now - started > CALL_TIMEOUT_SEC:
                future.cancel()
                self._pending.pop(key, None)
                button, spec = self._buttons[key]
                button.setEnabled(True)
                self._append_log(
                    spec.label, False,
                    '{:.0f}초 안에 응답이 없어 포기했습니다'.format(CALL_TIMEOUT_SEC))

        for key, (button, spec) in self._buttons.items():
            if key in self._pending:
                continue
            ready = self._client(spec.service, spec.type_name).service_is_ready()
            if button.isEnabled() != ready:
                button.setEnabled(ready)
                if not ready:
                    button.setToolTip('{}\n(서비스가 없습니다)'.format(spec.service))

    def _append_log(self, label, success, message):
        mark = 'OK  ' if success else 'FAIL'
        stamp = time.strftime('%H:%M:%S')
        self._log.appendPlainText(
            '[{}] {} {} -- {}'.format(stamp, mark, label, message))

    # ---------------------------------------------------------------- 정리

    def shutdown(self):
        self._timer.stop()
        if self._status_sub is not None:
            self._node.destroy_subscription(self._status_sub)
        for client in list(self._clients.values()) + list(self._param_clients.values()):
            self._node.destroy_client(client)
        self._clients.clear()
        self._param_clients.clear()
