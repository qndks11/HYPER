#!/usr/bin/env python3
# =====================================================================
# GPS 정확도 + 위치/방위 모니터 GUI
#
# 세 토픽을 한 창에 모은다:
#   /ublox_gps_node/navpvt   (ublox_msgs/NavPVT)  -> hAcc/vAcc, fix, RTK, 위성수
#   /odometry/gps            (nav_msgs/Odometry)  -> GPS만으로 푼 map 좌표 x/y
#   /odometry/filtered_map   (nav_msgs/Odometry)  -> EKF 융합 map 좌표 x/y + yaw
#   /imu                     (sensor_msgs/Imu)    -> E2BOX EBIMU-9DOFV5 링크 상태
#
# x/y를 두 벌 다 띄우는 이유: navsat_transform이 내는 /odometry/gps는 GPS만의
# 답이고 ekf_global이 내는 /odometry/filtered_map은 IMU/엔코더까지 섞은 답이라,
# 둘이 벌어지는 정도가 곧 추측항법 드리프트다. 한쪽만 보면 EKF가 발산해도
# 눈치채기 어렵다.
#
# 링크 상태는 hyper_ebimu 드라이버가 따로 토픽으로 내주지 않는다(연결/끊김을 로그로만
# 찍는다). 그래서 "드라이버가 내는 토픽이 지금 흐르고 있는가"로 대신 본다 -- USB-UART가
# 빠지거나 센서가 멈추면 드라이버는 재연결될 때까지 아무것도 publish하지 않으므로
# 실질적으로 같은 신호다.
#
# hAcc/vAcc는 NavPVT에서만 온다. NavSatFix(/gps/fix)의 position_covariance로
# hAcc는 역산되지만(covariance = hAcc^2) vAcc/fix_type/num_sv/RTK 비트는 없다.
#
# yaw: EBIMU-9DOFV5는 지자기 융합 9축이라 켜는 순간부터 절대 방위가 있고, ekf_global이
# 그 yaw를 그대로 먹는다(dual_ekf_navsat.yaml). 그래서 초기 yaw 캘리브레이션 절차는
# 없앴다. 나침반은 EKF yaw(/odometry/filtered_map, GPS가 끊겨도 odom으로 propagate됨)를
# 그린다.
#
# 실행:
#   ros2 run hyper_localization gps_accuracy_gui.py
#   ros2 run hyper_localization gps_accuracy_gui.py --ros-args -p navpvt_topic:=/other/navpvt
# =====================================================================

import math
import signal
import sys
import threading
import time

from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from ublox_msgs.msg import NavPVT

from PyQt5.QtCore import Qt, QPointF, QRectF, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QApplication, QFrame, QGridLayout, QHBoxLayout, QLabel, QVBoxLayout, QWidget)

FIX_TYPES = {
    0: 'NO FIX',
    1: 'DEAD RECKONING',
    2: '2D FIX',
    3: '3D FIX',
    4: 'GNSS + DR',
    5: 'TIME ONLY',
}

# NavPVT.flags 상위 2비트 (FLAGS_CARRIER_PHASE_MASK = 192)
CARRIER_PHASE = {0: '---', 64: 'RTK FLOAT', 128: 'RTK FIXED'}

# hAcc(m) 색 구간. RTK FIXED면 보통 0.02m 근처, FLOAT면 0.3~1m,
# 단독측위(3D FIX)면 1~3m 정도가 나온다.
GOOD_M = 0.10
OK_M = 1.00

COLOR_GOOD = '#3fb950'
COLOR_OK = '#d29922'
COLOR_BAD = '#f85149'
COLOR_STALE = '#6e7681'
COLOR_TEXT = '#c9d1d9'
COLOR_BG = '#0d1117'
COLOR_GPS = '#58a6ff'    # /odometry/gps 계열
COLOR_EKF = '#bc8cff'    # /odometry/filtered_map 계열

STALE_MS = 2000

# IMU 링크는 GPS보다 훨씬 빠르게(수십~수백 Hz) 도는 토픽이라 잠깐만 끊겨도 바로
# 티가 난다. 1초면 재연결 대기(hyper_ebimu의 reconnect_wait_seconds 기본 2초)보다
# 짧아서 끊김을 곧바로 잡아내면서도 시리얼 지터에 오탐하지 않는다.
IMU_STALE_S = 1.0
IMU_POLL_MS = 500
# Hz는 여러 폴링 구간에 걸쳐 평균낸다. 시리얼 버퍼에 몰려 있던 샘플이 한꺼번에
# 올라오면 500ms 창 하나만으로는 같은 100Hz 센서가 27Hz -> 144Hz로 튄다.
IMU_RATE_WINDOW = 4

def format_accuracy(metres):
    """hAcc/vAcc를 창 폭 안에 들어가는 길이로 찍는다.

    fix가 없을 때 u-blox는 hAcc로 수십 km(수천만 mm)를 내보내므로 항상 .3f로
    찍으면 40pt 글씨가 창 밖으로 잘려 나간다. 크기에 따라 자릿수를 줄인다.
    """
    if metres < 10.0:
        return f'{metres:.3f} m'
    if metres < 1000.0:
        return f'{metres:.1f} m'
    return f'{metres / 1000.0:.1f} km'


def accuracy_color(metres):
    if metres is None:
        return COLOR_STALE
    if metres <= GOOD_M:
        return COLOR_GOOD
    if metres <= OK_M:
        return COLOR_OK
    return COLOR_BAD


def yaw_from_quaternion(q):
    """쿼터니언 -> ENU yaw [rad]. tf_transformations 의존성을 안 만들려고 직접 푼다."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CompassWidget(QWidget):
    """yaw를 나침반 바늘로 그린다.

    입력 yaw는 REP-103 ENU(0=East, 반시계 +)이고, 사람이 읽는 건 방위각
    (0=North, 시계 +)이라 bearing = (90 - yaw_deg) mod 360 으로 바꿔 그린다.
    화면은 North가 위.
    """

    def __init__(self, diameter=150):
        super().__init__()
        self._yaw = None
        self.setFixedSize(diameter, diameter)

    def set_yaw(self, yaw_rad):
        self._yaw = yaw_rad
        self.update()

    def bearing_deg(self):
        if self._yaw is None:
            return None
        return (90.0 - math.degrees(self._yaw)) % 360.0

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        side = min(self.width(), self.height())
        cx, cy = self.width() / 2.0, self.height() / 2.0
        r = side / 2.0 - 12.0
        live = self._yaw is not None
        ring = QColor(COLOR_TEXT if live else COLOR_STALE)

        p.setPen(QPen(ring, 2))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QRectF(cx - r, cy - r, 2 * r, 2 * r))

        # 30도마다 눈금, 90도마다 길게
        for deg in range(0, 360, 30):
            a = math.radians(deg)
            inner = r - (9 if deg % 90 == 0 else 5)
            p.setPen(QPen(ring, 2 if deg % 90 == 0 else 1))
            p.drawLine(QPointF(cx + inner * math.sin(a), cy - inner * math.cos(a)),
                       QPointF(cx + r * math.sin(a), cy - r * math.cos(a)))

        p.setFont(QFont('DejaVu Sans', 9, QFont.Bold))
        for label, deg in (('N', 0), ('E', 90), ('S', 180), ('W', 270)):
            a = math.radians(deg)
            tx = cx + (r + 7) * math.sin(a)
            ty = cy - (r + 7) * math.cos(a)
            p.setPen(QPen(QColor(COLOR_BAD if label == 'N' else
                                 (COLOR_TEXT if live else COLOR_STALE))))
            p.drawText(QRectF(tx - 10, ty - 9, 20, 18), Qt.AlignCenter, label)

        if not live:
            p.setPen(QPen(QColor(COLOR_STALE)))
            p.setFont(QFont('DejaVu Sans', 8))
            p.drawText(QRectF(cx - 40, cy - 8, 80, 16), Qt.AlignCenter, 'no yaw')
            return

        # 바늘: 진행 방향은 채운 삼각형, 꼬리는 짧은 반대편 선
        a = math.radians(self.bearing_deg())
        tip = QPointF(cx + (r - 14) * math.sin(a), cy - (r - 14) * math.cos(a))
        left = QPointF(cx + 9 * math.sin(a + math.pi * 0.5),
                       cy - 9 * math.cos(a + math.pi * 0.5))
        right = QPointF(cx + 9 * math.sin(a - math.pi * 0.5),
                        cy - 9 * math.cos(a - math.pi * 0.5))
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(COLOR_EKF))
        p.drawPolygon(QPolygonF([tip, left, right]))

        tail = QPointF(cx - (r * 0.45) * math.sin(a), cy + (r * 0.45) * math.cos(a))
        p.setPen(QPen(QColor(COLOR_EKF), 2))
        p.drawLine(QPointF(cx, cy), tail)


class ImuLinkMonitor:
    """드라이버 원본 IMU 토픽의 수신 유무/주기를 센다.

    Imu 메시지는 초당 수십~수백 개가 오므로 위젯 갱신을 메시지마다 signal로
    던지면 Qt 이벤트 루프가 그것만 처리하게 된다. ROS 콜백에서는 카운터만
    올리고, Qt 쪽 QTimer가 IMU_POLL_MS마다 여기서 값을 꺼내 그린다.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._count = 0
        self._last_rx = None          # 마지막 수신 시각 (monotonic)
        self._window_start = time.monotonic()
        self._buckets = deque(maxlen=IMU_RATE_WINDOW)   # (개수, 구간 길이)

    def on_message(self):
        now = time.monotonic()
        with self._lock:
            self._count += 1
            self._last_rx = now

    def sample(self):
        """(hz, age_s) 반환. 한 번도 못 받았으면 age_s는 None."""
        now = time.monotonic()
        with self._lock:
            elapsed = now - self._window_start
            self._buckets.append((self._count, elapsed))
            self._count = 0
            self._window_start = now
            last_rx = self._last_rx
            total = sum(c for c, _ in self._buckets)
            span = sum(e for _, e in self._buckets)
        hz = total / span if span > 0.0 else 0.0
        return hz, (None if last_rx is None else now - last_rx)


class GpsAccuracyWindow(QWidget):
    """ROS 콜백은 rclpy 스레드에서 오므로 위젯을 직접 못 만진다.
    pyqtSignal로 넘겨 Qt 메인 스레드에서만 갱신한다."""

    navpvt = pyqtSignal(object)
    gps_odom = pyqtSignal(object)
    ekf_odom = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self.setWindowTitle('HYPER GPS Accuracy')
        self.setMinimumWidth(620)

        self._h_acc = self._big_value()
        self._v_acc = self._big_value()
        self._status = QLabel('waiting for data...')
        self._detail = QLabel('')
        self._compass = CompassWidget()
        self._yaw_text = QLabel('--')
        self._gps_xy = QLabel('--')
        self._ekf_xy = QLabel('--')
        self._delta = QLabel('--')
        self._imu_link = QLabel('--')
        self._source = QLabel('')

        self._imu_monitor = None
        self._ekf_yaw = None

        root = QVBoxLayout(self)
        root.setContentsMargins(24, 18, 24, 16)
        root.setSpacing(12)

        acc = QGridLayout()
        acc.setHorizontalSpacing(28)
        acc.addWidget(self._caption('Horizontal (hAcc)'), 0, 0)
        acc.addWidget(self._caption('Vertical (vAcc)'), 0, 1)
        acc.addWidget(self._h_acc, 1, 0)
        acc.addWidget(self._v_acc, 1, 1)
        root.addLayout(acc)

        self._status.setFont(QFont('DejaVu Sans', 15, QFont.Bold))
        self._status.setAlignment(Qt.AlignCenter)
        root.addWidget(self._status)

        self._detail.setFont(QFont('DejaVu Sans Mono', 10))
        self._detail.setAlignment(Qt.AlignCenter)
        self._detail.setStyleSheet(f'color: {COLOR_STALE};')
        root.addWidget(self._detail)

        root.addWidget(self._divider())

        imu_row = QHBoxLayout()
        imu_row.setSpacing(10)
        imu_caption = self._caption('IMU link  E2BOX EBIMU-9DOFV5 (USB-UART)')
        imu_caption.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        imu_row.addWidget(imu_caption)
        self._imu_link.setFont(QFont('DejaVu Sans Mono', 11, QFont.Bold))
        self._imu_link.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self._imu_link.setStyleSheet(f'color: {COLOR_STALE};')
        imu_row.addWidget(self._imu_link, 1)
        root.addLayout(imu_row)

        root.addWidget(self._divider())

        # 아래쪽: 왼쪽 나침반, 오른쪽 좌표 3줄
        bottom = QHBoxLayout()
        bottom.setSpacing(22)

        compass_col = QVBoxLayout()
        compass_col.setSpacing(4)
        compass_col.addWidget(self._compass, alignment=Qt.AlignCenter)
        self._yaw_text.setFont(QFont('DejaVu Sans Mono', 10, QFont.Bold))
        self._yaw_text.setAlignment(Qt.AlignCenter)
        self._yaw_text.setStyleSheet(f'color: {COLOR_EKF};')
        compass_col.addWidget(self._yaw_text)
        bottom.addLayout(compass_col)

        xy_col = QGridLayout()
        xy_col.setVerticalSpacing(6)
        xy_col.setHorizontalSpacing(10)
        for row, (cap, widget, colour) in enumerate((
                ('GPS only   /odometry/gps', self._gps_xy, COLOR_GPS),
                ('EKF fused  /odometry/filtered_map', self._ekf_xy, COLOR_EKF),
                ('separation  |EKF - GPS|', self._delta, COLOR_TEXT))):
            label = self._caption(cap)
            label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            widget.setFont(QFont('DejaVu Sans Mono', 13, QFont.Bold))
            widget.setStyleSheet(f'color: {colour};')
            xy_col.addWidget(label, row * 2, 0)
            xy_col.addWidget(widget, row * 2 + 1, 0)
        bottom.addLayout(xy_col, 1)
        root.addLayout(bottom)

        self._source.setFont(QFont('DejaVu Sans Mono', 8))
        self._source.setAlignment(Qt.AlignCenter)
        self._source.setStyleSheet(f'color: {COLOR_STALE};')
        root.addWidget(self._source)

        self.setStyleSheet(f'background: {COLOR_BG}; color: {COLOR_TEXT};')

        self.navpvt.connect(self._render_navpvt)
        self.gps_odom.connect(self._render_gps_odom)
        self.ekf_odom.connect(self._render_ekf_odom)

        self._gps_pos = None
        self._ekf_pos = None

        # 수신이 끊겨도 마지막 값이 그대로 남아 있으면 오해하기 쉬우므로,
        # 토픽별로 STALE_MS 넘게 조용하면 회색으로 떨어뜨린다.
        self._timers = {
            'navpvt': self._stale_timer(self._navpvt_stale),
            'gps': self._stale_timer(self._gps_stale),
            'ekf': self._stale_timer(self._ekf_stale),
        }

        self._imu_timer = QTimer(self)
        self._imu_timer.timeout.connect(self._render_imu_link)
        self._imu_timer.setInterval(IMU_POLL_MS)
        self._imu_timer.start()

        self._refresh_yaw()

    # ---------- 위젯 헬퍼 ----------
    def _caption(self, text):
        label = QLabel(text)
        label.setFont(QFont('DejaVu Sans', 9))
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet(f'color: {COLOR_STALE};')
        return label

    def _big_value(self):
        label = QLabel('--')
        label.setFont(QFont('DejaVu Sans', 30, QFont.Bold))
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet(f'color: {COLOR_STALE};')
        return label

    def _divider(self):
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet(f'color: #21262d; background: #21262d; max-height: 1px;')
        return line

    def _stale_timer(self, slot):
        timer = QTimer(self)
        timer.timeout.connect(slot)
        timer.setInterval(STALE_MS)
        timer.start()
        return timer

    def set_topics(self, navpvt, gps, ekf, imu):
        self._source.setText(f'{navpvt}   |   {gps}   |   {ekf}   |   {imu}')

    def set_imu_monitor(self, monitor):
        self._imu_monitor = monitor

    def _refresh_yaw(self):
        """EBIMU-9DOFV5는 9축(지자기 융합)이라 켜는 순간부터 절대 yaw가 있고,
        ekf_global이 그 yaw를 먹는다. 그래서 따로 "확립됐는가"를 따지지 않고
        라이브 EKF yaw(/odometry/filtered_map)를 그대로 그린다."""
        if self._ekf_yaw is None:
            self._compass.set_yaw(None)
            self._yaw_text.setText('--\nno /odometry/filtered_map')
            self._yaw_text.setStyleSheet(f'color: {COLOR_STALE};')
            return

        self._compass.set_yaw(self._ekf_yaw)
        self._yaw_text.setText(
            f'{math.degrees(self._ekf_yaw):+7.1f}d ENU  (live)\n'
            f'{self._compass.bearing_deg():6.1f}d bearing')
        self._yaw_text.setStyleSheet(f'color: {COLOR_EKF};')

    # ---------- stale ----------
    def _navpvt_stale(self):
        self._h_acc.setStyleSheet(f'color: {COLOR_STALE};')
        self._v_acc.setStyleSheet(f'color: {COLOR_STALE};')
        self._status.setText('NO DATA (stale)')
        self._status.setStyleSheet(f'color: {COLOR_STALE};')

    def _gps_stale(self):
        self._gps_pos = None
        self._gps_xy.setText('no fix / no data')
        self._gps_xy.setStyleSheet(f'color: {COLOR_STALE};')
        self._update_delta()

    def _ekf_stale(self):
        self._ekf_pos = None
        self._ekf_xy.setText('no data')
        self._ekf_xy.setStyleSheet(f'color: {COLOR_STALE};')
        self._ekf_yaw = None
        self._refresh_yaw()
        self._update_delta()

    # ---------- 렌더 ----------
    def _render_navpvt(self, msg):
        self._timers['navpvt'].start()

        h_m = msg.h_acc / 1000.0   # mm -> m
        v_m = msg.v_acc / 1000.0
        self._h_acc.setText(format_accuracy(h_m))
        self._v_acc.setText(format_accuracy(v_m))
        self._h_acc.setStyleSheet(f'color: {accuracy_color(h_m)};')
        self._v_acc.setStyleSheet(f'color: {accuracy_color(v_m)};')

        fix = FIX_TYPES.get(msg.fix_type, f'? ({msg.fix_type})')
        rtk = CARRIER_PHASE.get(msg.flags & 192, '---')
        fix_ok = bool(msg.flags & 1)   # FLAGS_GNSS_FIX_OK

        if not fix_ok or msg.fix_type == 0:
            self._status.setText(f'{fix}  --  solution not usable')
            self._status.setStyleSheet(f'color: {COLOR_BAD};')
        elif rtk == 'RTK FIXED':
            self._status.setText(f'{fix}  --  {rtk}')
            self._status.setStyleSheet(f'color: {COLOR_GOOD};')
        elif rtk == 'RTK FLOAT':
            self._status.setText(f'{fix}  --  {rtk}')
            self._status.setStyleSheet(f'color: {COLOR_OK};')
        else:
            self._status.setText(fix)
            self._status.setStyleSheet(f'color: {COLOR_OK};')

        # p_dop는 0.01 단위, head_acc는 1e-5 deg 단위.
        self._detail.setText(
            f'sats {msg.num_sv:2d}   pDOP {msg.p_dop / 100.0:.2f}   '
            f'speed {msg.g_speed / 1000.0:.2f} m/s   '
            f'headAcc {msg.head_acc / 1e5:.1f}deg')

    def _render_gps_odom(self, msg):
        self._timers['gps'].start()
        p = msg.pose.pose.position
        self._gps_pos = (p.x, p.y)
        self._gps_xy.setText(f'x {p.x:10.3f}   y {p.y:10.3f}')
        self._gps_xy.setStyleSheet(f'color: {COLOR_GPS};')
        self._update_delta()

    def _render_ekf_odom(self, msg):
        self._timers['ekf'].start()
        p = msg.pose.pose.position
        self._ekf_pos = (p.x, p.y)
        self._ekf_xy.setText(f'x {p.x:10.3f}   y {p.y:10.3f}')
        self._ekf_xy.setStyleSheet(f'color: {COLOR_EKF};')

        self._ekf_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self._refresh_yaw()
        self._update_delta()

    def _render_imu_link(self):
        if self._imu_monitor is None:
            return
        hz, age = self._imu_monitor.sample()
        if age is None:
            self._imu_link.setText('NO DATA -- never received')
            self._imu_link.setStyleSheet(f'color: {COLOR_STALE};')
        elif age > IMU_STALE_S:
            # 드라이버가 살아 있어도 시리얼이 끊기면 publish가 멈춘다. 재연결까지는
            # hyper_ebimu의 reconnect_wait_seconds(기본 2초)가 걸린다.
            self._imu_link.setText(f'DISCONNECTED -- silent {age:.1f} s')
            self._imu_link.setStyleSheet(f'color: {COLOR_BAD};')
        else:
            self._imu_link.setText(f'CONNECTED   {hz:5.1f} Hz')
            self._imu_link.setStyleSheet(f'color: {COLOR_GOOD};')

    def _update_delta(self):
        if self._gps_pos is None or self._ekf_pos is None:
            self._delta.setText('--')
            self._delta.setStyleSheet(f'color: {COLOR_STALE};')
            return
        dx = self._ekf_pos[0] - self._gps_pos[0]
        dy = self._ekf_pos[1] - self._gps_pos[1]
        dist = math.hypot(dx, dy)
        self._delta.setText(f'{dist:.3f} m')
        self._delta.setStyleSheet(f'color: {accuracy_color(dist)};')


class GpsAccuracyNode(Node):
    def __init__(self, window):
        super().__init__('gps_accuracy_gui')

        def param(name, default):
            return self.declare_parameter(
                name, default).get_parameter_value().string_value

        self.navpvt_topic = param('navpvt_topic', '/ublox_gps_node/navpvt')
        self.gps_topic = param('gps_odom_topic', '/odometry/gps')
        self.ekf_topic = param('ekf_odom_topic', '/odometry/filtered_map')
        # hyper_ebimu(EBIMU-9DOFV5)가 내는 토픽. 실차/시뮬레이션 모두 EKF가 먹는
        # /imu 그대로다 -- 이 토픽이 흐르는지로 센서 링크 상태를 본다.
        self.imu_topic = param('imu_topic', '/imu')

        # ublox_firmware7plus.hpp는 create_publisher<NavPVT>("~/navpvt", 1) --
        # 기본 QoS(RELIABLE/KEEP_LAST)라 여기서도 기본값으로 맞춘다.
        self.create_subscription(
            NavPVT, self.navpvt_topic, lambda m: window.navpvt.emit(m), 10)
        self.create_subscription(
            Odometry, self.gps_topic, lambda m: window.gps_odom.emit(m), 10)
        self.create_subscription(
            Odometry, self.ekf_topic, lambda m: window.ekf_odom.emit(m), 10)

        # 드라이버는 create_publisher<Imu>(topic, 10) -- 기본 RELIABLE이라 여기도
        # 기본 QoS로 맞춘다. 콜백은 카운터만 올린다(ImuLinkMonitor 주석 참고).
        self.imu_monitor = ImuLinkMonitor()
        self.create_subscription(
            Imu, self.imu_topic, lambda _m: self.imu_monitor.on_message(), 10)

        self.get_logger().info(
            f'GPS GUI: {self.navpvt_topic} | {self.gps_topic} | {self.ekf_topic} | '
            f'{self.imu_topic}')

def main():
    rclpy.init(args=sys.argv)
    app = QApplication(sys.argv)
    # Qt가 SIGINT를 삼키므로 Ctrl-C가 먹도록 기본 핸들러로 되돌린다.
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    window = GpsAccuracyWindow()
    node = GpsAccuracyNode(window)
    window.set_topics(node.navpvt_topic, node.gps_topic, node.ekf_topic,
                      node.imu_topic)
    window.set_imu_monitor(node.imu_monitor)
    window.show()

    # rclpy는 별도 스레드에서 spin하고 Qt가 메인 스레드를 잡는다.
    def spin():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass          # launch가 SIGTERM으로 내리는 정상 종료 경로
        finally:
            QApplication.quit()   # ROS가 죽으면 창도 같이 닫는다

    threading.Thread(target=spin, daemon=True).start()

    try:
        code = app.exec_()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
