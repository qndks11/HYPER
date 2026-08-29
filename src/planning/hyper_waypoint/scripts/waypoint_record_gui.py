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
# 호출:
#   <recorder>/start, <recorder>/stop  (std_srvs/Trigger)
#
#   ros2 run hyper_waypoint waypoint_record_gui.py
#   ros2 run hyper_waypoint waypoint_record_gui.py --ros-args -p recorder:=/other_recorder
# =====================================================================

import signal
import sys
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import Path
from std_msgs.msg import String
from std_srvs.srv import Trigger

from PyQt5.QtCore import Qt, QPointF, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication, QFrame, QGridLayout, QHBoxLayout, QLabel, QPushButton,
    QVBoxLayout, QWidget)

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


class PathView(QWidget):
    """지금까지 찍힌 점 + 현재 위치를 위에서 내려다본 미니맵.

    matplotlib을 쓰지 않는 이유는 이 창이 주행 중에 계속 떠 있기 때문입니다 --
    QPainter로 점만 찍으면 CPU를 거의 안 씁니다.
    """

    def __init__(self):
        super().__init__()
        self.setMinimumHeight(260)
        self._points = []       # [(x, y)] map 프레임
        self._car = None        # (x, y)

    def set_points(self, points):
        self._points = points
        self.update()

    def set_car(self, xy):
        self._car = xy
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(COLOR_BG))

        pts = list(self._points)
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


class RecorderGui(QWidget):
    status_arrived = pyqtSignal(str)
    path_arrived = pyqtSignal(object)

    def __init__(self, node):
        super().__init__()
        self._node = node
        self._recording = False

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

        # status가 한동안 끊기면 stale로 표시하기 위한 워치독.
        self._stale_timer = QTimer(self)
        self._stale_timer.timeout.connect(self._mark_stale)
        self._stale_timer.setSingleShot(True)

    # ------------------------------------------------------------ 서비스
    def _call(self, which):
        client = self._node.start_client if which == 'start' else self._node.stop_client
        if not client.service_is_ready():
            self._message.setText(f'{which} 서비스가 아직 없습니다 (레코더 노드 확인)')
            self._message.setStyleSheet(f'color: {COLOR_BAD};')
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(self._on_service_done)

    def _on_service_done(self, future):
        try:
            response = future.result()
        except Exception as exc:                      # noqa: BLE001 - 표시가 목적
            text, color = f'서비스 실패: {exc}', COLOR_BAD
        else:
            text = response.message
            color = COLOR_GOOD if response.success else COLOR_OK
        # 시그널을 하나 더 만들지 않고 QTimer로 GUI 스레드에 태웁니다.
        QTimer.singleShot(0, lambda: (
            self._message.setText(text),
            self._message.setStyleSheet(f'color: {color};')))

    # ------------------------------------------------------------- 표시
    def _mark_stale(self):
        self._state.setText('상태 없음')
        self._state.setStyleSheet(f'color: {COLOR_STALE};')

    def _on_status(self, text):
        self._stale_timer.start(int(STALE_S * 1000))
        status = parse_status(text)

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

        self.get_logger().info(f'watching {recorder}/status, {recorder}/path')

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
