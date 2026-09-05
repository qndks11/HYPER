#!/usr/bin/env python3
# =====================================================================
# 웨이포인트 녹화 조작판.
#
# waypoint_recorder_node를 auto_start:=false로 띄워 두고, 이 창에서 Record/Stop을
# 누릅니다. 녹화를 시작하는 순간이 곧 CSV를 truncate하는 순간이므로, 버튼을 누르기
# 전까지는 이전 녹화본이 그대로 남아 있습니다.
#
# 구독:
#   <recorder>/status  (std_msgs/String)  key=value 한 줄 -- 5Hz + 점이 찍힐 때마다
#   <recorder>/path    (nav_msgs/Path)    지금까지 찍힌 점 전부
# 읽기:
#   저장 파일 칸이 가리키는 CSV -- 이전 녹화본을 미니맵에 흐리게 깔아 둡니다.
# 호출:
#   <recorder>/start, <recorder>/stop  (std_srvs/Trigger)
#
#   ros2 run hyper_waypoint waypoint_record_gui.py
#   ros2 run hyper_waypoint waypoint_record_gui.py --ros-args -p recorder:=/other_recorder
# =====================================================================

import csv
import os
import signal
import sys
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import Path
from std_msgs.msg import String
from std_srvs.srv import Trigger

from PyQt5.QtCore import Qt, QPointF, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication, QFrame, QGridLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QVBoxLayout, QWidget)

# gps_accuracy_gui.py와 같은 팔레트를 씁니다 -- 실차에서 두 창을 나란히 띄우므로.
COLOR_TEXT = '#c9d1d9'
COLOR_BG = '#0d1117'
COLOR_PANEL = '#161b22'
COLOR_GOOD = '#3fb950'
COLOR_OK = '#d29922'
COLOR_BAD = '#f85149'
COLOR_STALE = '#6e7681'
COLOR_PATH = '#58a6ff'
COLOR_CAR = '#bc8cff'
COLOR_PREV = '#8b949e'   # 이전 녹화본. 이번 녹화(COLOR_PATH)와 눈으로 구분되게 회색.

# /gps/fix의 NavSatStatus. 녹화 품질은 결국 이 값입니다.
GPS_STATUS = {
    -1: ('NO FIX', COLOR_BAD),
    0: ('단독측위', COLOR_OK),
    1: ('SBAS', COLOR_OK),
    2: ('RTK / GBAS', COLOR_GOOD),
}

STALE_S = 2.0


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
    except (KeyError, ValueError):
        return default


def load_csv_points(path):
    """녹화 CSV에서 (x, y)만 뽑아 옵니다. 못 읽으면 ([], 사유).

    레코더가 쓰는 헤더(idx,stamp_sec,x,y,...)를 이름으로 찾습니다 -- 열이 뒤에
    붙어도 안 깨지도록. 좌표가 비어 있는 줄(EKF 없이 기록된 줄)은 건너뜁니다.
    """
    if not path:
        return [], ''
    if not os.path.isfile(path):
        return [], '이전 녹화본 없음'
    try:
        with open(path, newline='', encoding='utf-8') as handle:
            reader = csv.DictReader(handle)
            if not reader.fieldnames or 'x' not in reader.fieldnames \
                    or 'y' not in reader.fieldnames:
                return [], 'CSV에 x,y 열이 없습니다'
            points = []
            for row in reader:
                try:
                    points.append((float(row['x']), float(row['y'])))
                except (TypeError, ValueError):
                    continue
    except OSError as exc:                             # noqa: BLE001 - 표시가 목적
        return [], f'이전 녹화본을 못 읽었습니다: {exc}'
    return points, ''


class PathView(QWidget):
    """지금까지 찍힌 점 + 현재 위치를 위에서 내려다본 미니맵.

    matplotlib을 쓰지 않는 이유는 이 창이 주행 중에 계속 떠 있기 때문입니다 --
    QPainter로 점만 찍으면 CPU를 거의 안 씁니다.
    """

    def __init__(self):
        super().__init__()
        self.setMinimumHeight(260)
        self._points = []       # [(x, y)] 이번 녹화, map 프레임
        self._previous = []     # [(x, y)] 파일에서 읽어 온 이전 녹화본
        self._car = None        # (x, y)

    def set_points(self, points):
        self._points = points
        self.update()

    def set_previous(self, points):
        self._previous = points
        self.update()

    def set_car(self, xy):
        self._car = xy
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(COLOR_BG))

        pts = list(self._previous) + list(self._points)
        if self._car is not None:
            pts.append(self._car)
        if not pts:
            painter.setPen(QColor(COLOR_STALE))
            painter.drawText(self.rect(), Qt.AlignCenter, '위치 없음')
            return

        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        # 한 점뿐일 때 0으로 나누지 않도록 최소 폭을 줍니다.
        span = max(max(xs) - min(xs), max(ys) - min(ys), 1.0)
        margin = 16.0
        scale = (min(self.width(), self.height()) - 2 * margin) / span
        cx = (max(xs) + min(xs)) / 2.0
        cy = (max(ys) + min(ys)) / 2.0

        def to_screen(x, y):
            # map은 X=East, Y=North. 화면은 Y가 아래로 증가하므로 뒤집습니다.
            return QPointF(
                self.width() / 2.0 + (x - cx) * scale,
                self.height() / 2.0 - (y - cy) * scale)

        # 이전 녹화본을 먼저 깔아 둡니다 -- 이번 녹화 선이 그 위에 그려지도록.
        if self._previous:
            painter.setPen(QPen(QColor(COLOR_PREV), 1))
            last = None
            for x, y in self._previous:
                point = to_screen(x, y)
                if last is not None:
                    painter.drawLine(last, point)
                last = point

        painter.setPen(QPen(QColor(COLOR_PATH), 2))
        previous = None
        for x, y in self._points:
            point = to_screen(x, y)
            if previous is not None:
                painter.drawLine(previous, point)
            previous = point
        painter.setBrush(QColor(COLOR_PATH))
        for x, y in self._points:
            painter.drawEllipse(to_screen(x, y), 2.0, 2.0)

        if self._car is not None:
            painter.setPen(QPen(QColor(COLOR_CAR), 2))
            painter.setBrush(QColor(COLOR_CAR))
            painter.drawEllipse(to_screen(*self._car), 5.0, 5.0)

        painter.setPen(QColor(COLOR_STALE))
        painter.drawText(8, self.height() - 8, f'{span:.0f} m')
        if self._previous:
            painter.setPen(QColor(COLOR_PREV))
            painter.drawText(
                8, 16, f'이전 녹화 {len(self._previous)} 점')


class RecorderGui(QWidget):
    status_arrived = pyqtSignal(str)
    path_arrived = pyqtSignal(object)
    # 아래 둘은 서비스/파라미터 콜백에서 emit됩니다 -- 그 콜백은 rclpy executor
    # 스레드에서 도는데, Qt 위젯과 QTimer는 GUI 스레드에서만 안전하므로 시그널로
    # 넘겨 받습니다. (QTimer.singleShot을 executor 스레드에서 부르면 조용히 무시됩니다.)
    param_applied = pyqtSignal()
    message_arrived = pyqtSignal(str, str)

    def __init__(self, node):
        super().__init__()
        self._node = node
        self._recording = False
        # 미니맵에 깔아 둔 이전 녹화본의 경로. None이면 아직 안 읽었다는 뜻이라
        # 다음 기회에 다시 읽습니다(같은 경로를 매 status마다 다시 읽지 않도록).
        self._previous_file = None
        # 레코더가 지금 들고 있는 output_csv. 저장 파일 칸이 비었을 때의 대상입니다.
        self._recorder_file = ''

        self.setWindowTitle('Waypoint Recorder')
        self.setStyleSheet(f'background-color: {COLOR_BG}; color: {COLOR_TEXT};')
        self.resize(520, 640)

        root = QVBoxLayout(self)

        self._state = QLabel('연결 대기')
        self._state.setAlignment(Qt.AlignCenter)
        self._state.setFont(QFont('DejaVu Sans', 34, QFont.Bold))
        root.addWidget(self._state)

        self._file = QLabel('-')
        self._file.setAlignment(Qt.AlignCenter)
        self._file.setStyleSheet(f'color: {COLOR_STALE};')
        self._file.setWordWrap(True)
        root.addWidget(self._file)

        # 저장할 CSV 경로. Record를 누르는 순간 레코더의 output_csv 파라미터로
        # 밀어 넣은 뒤 start를 부릅니다. 비워 두면 레코더가 이미 들고 있는 값을
        # 그대로 씁니다.
        name_row = QHBoxLayout()
        name_label = QLabel('저장 파일')
        name_label.setStyleSheet(f'color: {COLOR_STALE};')
        name_row.addWidget(name_label)
        self._name_edit = QLineEdit()
        self._name_edit.setPlaceholderText('waypoint_record.csv')
        initial_name = getattr(node, 'initial_filename', '')
        if initial_name:
            self._name_edit.setText(initial_name)
        self._name_edit.setFont(QFont('DejaVu Sans Mono', 12))
        self._name_edit.setStyleSheet(
            f'background-color: {COLOR_PANEL}; color: {COLOR_TEXT}; '
            f'border: 1px solid {COLOR_STALE}; border-radius: 4px; padding: 4px;')
        self._name_edit.returnPressed.connect(lambda: self._call('start'))
        # 파일 이름을 바꾸면 그 파일의 이전 녹화본을 미니맵에 깔아 줍니다 --
        # Record가 무엇을 덮어쓰는지 누르기 전에 보이도록.
        self._name_edit.editingFinished.connect(self._sync_previous)
        name_row.addWidget(self._name_edit, stretch=1)
        root.addLayout(name_row)

        buttons = QHBoxLayout()
        self._record_button = QPushButton('● Record')
        self._stop_button = QPushButton('■ Stop')
        for button in (self._record_button, self._stop_button):
            button.setFont(QFont('DejaVu Sans', 15, QFont.Bold))
            button.setMinimumHeight(56)
            buttons.addWidget(button)
        self._record_button.clicked.connect(lambda: self._call('start'))
        self._stop_button.clicked.connect(lambda: self._call('stop'))
        root.addLayout(buttons)

        grid_frame = QFrame()
        grid_frame.setStyleSheet(f'background-color: {COLOR_PANEL}; border-radius: 6px;')
        grid = QGridLayout(grid_frame)
        self._fields = {}
        for row, (key, caption) in enumerate([
            ('points', '기록된 점'),
            ('length', '누적 거리'),
            ('since_last', '다음 점까지'),
            ('pose', '현재 위치 (map)'),
            ('speed', '속도'),
            ('gps', 'GPS 상태'),
            ('cov', 'EKF 공분산 xx / yy'),
        ]):
            label = QLabel(caption)
            label.setStyleSheet(f'color: {COLOR_STALE};')
            value = QLabel('-')
            value.setFont(QFont('DejaVu Sans Mono', 13, QFont.Bold))
            grid.addWidget(label, row, 0)
            grid.addWidget(value, row, 1, alignment=Qt.AlignRight)
            self._fields[key] = value
        root.addWidget(grid_frame)

        self._view = PathView()
        root.addWidget(self._view, stretch=1)

        self._message = QLabel(' ')
        self._message.setAlignment(Qt.AlignCenter)
        self._message.setStyleSheet(f'color: {COLOR_STALE};')
        self._message.setWordWrap(True)
        root.addWidget(self._message)

        # ROS 콜백은 executor 스레드에서 옵니다. Qt 위젯은 GUI 스레드에서만 만질 수
        # 있으므로 시그널로 넘깁니다(gps_accuracy_gui.py와 같은 구조).
        self.status_arrived.connect(self._on_status)
        self.path_arrived.connect(self._view.set_points)
        self.param_applied.connect(self._start_after_param)
        self.message_arrived.connect(self._show_message)

        # status가 한동안 끊기면 stale로 표시하기 위한 워치독.
        self._stale_timer = QTimer(self)
        self._stale_timer.timeout.connect(self._mark_stale)
        self._stale_timer.setSingleShot(True)

        # 창이 뜨자마자 이전 녹화본을 보여 줍니다. status를 기다리지 않는 이유는
        # 레코더가 아직 안 떠 있어도 파일은 읽을 수 있기 때문입니다.
        self._sync_previous()

    # --------------------------------------------------- 이전 녹화본
    def _sync_previous(self):
        """저장 파일 칸(비었으면 레코더의 output_csv)의 CSV를 미니맵에 깝니다.

        녹화 중에는 그 파일이 지금 쓰이는 중이라 깔지 않습니다 -- 이번 녹화 선이
        곧 그 파일의 내용입니다.
        """
        if self._recording:
            return
        path = self._name_edit.text().strip() or self._recorder_file
        if path == self._previous_file:
            return
        self._previous_file = path
        points, problem = load_csv_points(path)
        self._view.set_previous(points)
        if problem:
            self._show_message(problem, COLOR_STALE)
        elif points:
            self._show_message(
                f'이전 녹화본 {len(points)} 점 -- Record를 누르면 덮어씁니다', COLOR_OK)

    # ------------------------------------------------------------ 서비스
    def _call(self, which):
        client = self._node.start_client if which == 'start' else self._node.stop_client
        if not client.service_is_ready():
            self._show_message(f'{which} 서비스가 아직 없습니다 (레코더 노드 확인)', COLOR_BAD)
            return
        if which == 'start':
            name = self._name_edit.text().strip()
            if name:
                # 파일 이름을 레코더 파라미터로 밀어 넣고, 반영되면 param_applied
                # 시그널을 통해 GUI 스레드에서 start를 부릅니다.
                if not self._node.set_output_csv(name, self._on_param_done):
                    self._show_message(
                        'set_parameters 서비스가 아직 없습니다 (레코더 노드 확인)', COLOR_BAD)
                return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(self._on_service_done)

    # executor 스레드에서 불립니다 -- Qt는 시그널로만 건드립니다.
    def _on_param_done(self, future):
        try:
            results = future.result().results
        except Exception as exc:                       # noqa: BLE001 - 표시가 목적
            self.message_arrived.emit(f'파일 이름 설정 실패: {exc}', COLOR_BAD)
            return
        if results and not results[0].successful:
            self.message_arrived.emit(
                f'파일 이름 거부됨: {results[0].reason or "rejected"}', COLOR_BAD)
            return
        self.param_applied.emit()

    def _start_after_param(self):
        future = self._node.start_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_service_done)

    # executor 스레드에서 불립니다 -- Qt는 시그널로만 건드립니다.
    def _on_service_done(self, future):
        try:
            response = future.result()
        except Exception as exc:                      # noqa: BLE001 - 표시가 목적
            self.message_arrived.emit(f'서비스 실패: {exc}', COLOR_BAD)
            return
        self.message_arrived.emit(
            response.message, COLOR_GOOD if response.success else COLOR_OK)

    def _show_message(self, text, color):
        self._message.setText(text)
        self._message.setStyleSheet(f'color: {color};')

    # ------------------------------------------------------------- 표시
    def _mark_stale(self):
        self._state.setText('상태 없음')
        self._state.setStyleSheet(f'color: {COLOR_STALE};')

    def _on_status(self, text):
        self._stale_timer.start(int(STALE_S * 1000))
        status = parse_status(text)

        was_recording = self._recording
        self._recording = status.get('recording') == '1'
        if self._recording:
            self._state.setText('● REC')
            self._state.setStyleSheet(f'color: {COLOR_BAD};')
        else:
            self._state.setText('IDLE')
            self._state.setStyleSheet(f'color: {COLOR_STALE};')
        self._record_button.setEnabled(not self._recording)
        self._stop_button.setEnabled(self._recording)

        self._file.setText(status.get('file', '-'))
        self._recorder_file = status.get('file', '')
        if self._recording:
            if not was_recording:
                # 녹화가 시작된 순간 파일은 truncate됐습니다. 깔아 둔 이전 녹화본은
                # 더 이상 그 파일의 내용이 아니므로 지웁니다.
                self._view.set_previous([])
            # 녹화 중이거나 막 끝난 파일은 이번 녹화 선이 곧 그 내용이므로, 읽은
            # 것으로 표시해 두어 정지 후에 같은 경로를 겹쳐 깔지 않게 합니다.
            self._previous_file = self._recorder_file
        else:
            self._sync_previous()

        points = status.get('points', '0')
        spacing = as_float(status, 'spacing')
        self._fields['points'].setText(
            f'{points} 점' + (f'  (간격 {spacing:.2f} m)' if spacing is not None else ''))

        length = as_float(status, 'length')
        self._fields['length'].setText('-' if length is None else f'{length:.1f} m')

        # 다음 점까지 남은 거리. 기록 중이 아니거나 첫 점 전에는 의미가 없습니다.
        since = as_float(status, 'since_last')
        if since is None or not self._recording:
            self._fields['since_last'].setText('-')
            self._fields['since_last'].setStyleSheet(f'color: {COLOR_STALE};')
        else:
            remaining = max((spacing or 0.0) - since, 0.0)
            self._fields['since_last'].setText(f'{since:.2f} m  (남은 {remaining:.2f} m)')
            self._fields['since_last'].setStyleSheet(
                f'color: {COLOR_GOOD if remaining <= 0.0 else COLOR_TEXT};')

        x = as_float(status, 'x')
        y = as_float(status, 'y')
        pose_age = as_float(status, 'pose_age', 0.0)
        if x is None or y is None:
            self._fields['pose'].setText('/odometry/filtered_map 없음')
            self._fields['pose'].setStyleSheet(f'color: {COLOR_BAD};')
            self._view.set_car(None)
        else:
            fresh = pose_age is not None and pose_age <= STALE_S
            self._fields['pose'].setText(f'{x:.2f}, {y:.2f}')
            self._fields['pose'].setStyleSheet(
                f'color: {COLOR_TEXT if fresh else COLOR_STALE};')
            self._view.set_car((x, y))

        speed = as_float(status, 'speed')
        self._fields['speed'].setText('-' if speed is None else f'{speed:+.2f} m/s')

        gps_status = status.get('gps_status')
        gps_age = as_float(status, 'gps_age')
        if gps_status is None:
            self._fields['gps'].setText('/gps/fix 없음')
            self._fields['gps'].setStyleSheet(f'color: {COLOR_BAD};')
        else:
            caption, color = GPS_STATUS.get(
                int(gps_status), (f'? ({gps_status})', COLOR_OK))
            if gps_age is not None and gps_age > STALE_S:
                caption, color = f'{caption} (stale {gps_age:.0f}s)', COLOR_STALE
            self._fields['gps'].setText(caption)
            self._fields['gps'].setStyleSheet(f'color: {color};')

        cov_xx = as_float(status, 'cov_xx')
        cov_yy = as_float(status, 'cov_yy')
        if cov_xx is None or cov_yy is None:
            self._fields['cov'].setText('-')
        else:
            self._fields['cov'].setText(f'{cov_xx:.3f} / {cov_yy:.3f}')
            self._fields['cov'].setStyleSheet(
                f'color: {COLOR_GOOD if max(cov_xx, cov_yy) < 0.1 else COLOR_OK};')


class RecorderGuiNode(Node):
    def __init__(self):
        super().__init__('waypoint_record_gui')
        recorder = self.declare_parameter('recorder', '/waypoint_recorder').value
        recorder = recorder.rstrip('/')

        # 저장 파일 칸의 초기값. 보통 launch가 넘긴 waypoint_csv와 같은 값을 넣어,
        # 창이 뜨자마자 어디로 녹화되는지 보이게 합니다. 비어 있으면 칸도 비어 있고,
        # 그 경우 Record는 레코더가 이미 들고 있는 output_csv로 갑니다.
        self.initial_filename = self.declare_parameter('filename', '').value

        # 레코더의 status/path는 transient_local이라, 이 창을 나중에 띄워도
        # 마지막 상태와 지금까지 찍힌 경로를 그대로 받습니다.
        latched = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.gui = None
        self.create_subscription(
            String, f'{recorder}/status', self._on_status, latched)
        self.create_subscription(
            Path, f'{recorder}/path', self._on_path, latched)
        self.start_client = self.create_client(Trigger, f'{recorder}/start')
        self.stop_client = self.create_client(Trigger, f'{recorder}/stop')

        # 레코더 노드의 output_csv 파라미터를 GUI에서 바꾸기 위한 클라이언트.
        # recorder 네임스페이스가 곧 레코더 노드의 완전한 이름입니다.
        self.param_client = self.create_client(
            SetParameters, f'{recorder}/set_parameters')

        self.get_logger().info(f'watching {recorder}/status, {recorder}/path')

    def set_output_csv(self, path, done_callback):
        """레코더의 output_csv를 path로 설정. 서비스가 없으면 False."""
        if not self.param_client.service_is_ready():
            return False
        request = SetParameters.Request()
        request.parameters = [Parameter(
            name='output_csv',
            value=ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value=path))]
        future = self.param_client.call_async(request)
        future.add_done_callback(done_callback)
        return True

    def _on_status(self, msg):
        if self.gui is not None:
            self.gui.status_arrived.emit(msg.data)

    def _on_path(self, msg):
        if self.gui is not None:
            self.gui.path_arrived.emit(
                [(p.pose.position.x, p.pose.position.y) for p in msg.poses])


def main():
    rclpy.init()
    node = RecorderGuiNode()

    app = QApplication(sys.argv)
    gui = RecorderGui(node)
    node.gui = gui
    gui.show()

    # Ctrl-C와 launch의 종료 신호가 Qt 이벤트 루프를 뚫고 들어오도록. 파이썬 핸들러는
    # 이벤트 루프가 블록된 동안 돌지 않으므로 기본 동작(즉시 종료)으로 되돌립니다.
    # SIGTERM까지 넣는 이유: 이게 없으면 launch가 스택을 내릴 때 이 창만 안 죽고
    # 남아서, 다음 실행에서 레코더 서비스에 붙은 유령 GUI가 됩니다.
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)

    thread = threading.Thread(target=_spin, args=(node,), daemon=True)
    thread.start()
    try:
        app.exec_()
    finally:
        rclpy.try_shutdown()


def _spin(node):
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


if __name__ == '__main__':
    main()
