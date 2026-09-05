#!/usr/bin/env python3
# =====================================================================
# RTK moving-base 헤딩(절대방위) 모니터 GUI
#
# 두 토픽을 한 창에 모은다:
#   /ublox_gps_node_rover/navrelposned  (ublox_msgs/NavRELPOSNED9) -- 원본
#     relPosHeading/accHeading/flags. hp_pos_rec_product.cpp가 여기서
#     ekf_global이 먹는 imu/heading을 만든다(hyper_rtk/README.md 참고).
#   /imu/heading                        (sensor_msgs/Imu, yaw만 유효) -- 그
#     변환 결과. dual_ekf_navsat.yaml의 imu1이 이걸 절대 yaw로 먹는다.
#
# 왜 두 개를 같이 보는가
# -----------------------
# hp_pos_rec_product.cpp(콜백)는 FLAGS_REL_POS_HEAD_VALID가 꺼져 있어도
# imu/heading을 "일단" 계속 publish한다 -- 다만 orientation_covariance[8]을
# 1000 rad^2(사실상 "믿지 마라")로 채울 뿐이다. 그래서 imu/heading 토픽만
# 보면 RTK가 아직 fix를 못 잡았을 때도 그럴듯한 숫자가 계속 흘러나오는 것처럼
# 보인다 -- 실제로 유효한지는 navrelposned의 flags를 봐야 안다. 이 GUI는
# 그 flags를 그대로 보여주고, 두 토픽의 방위각이 서로 일치하는지(변환 버그
# 없이 그대로 전달됐는지)까지 함께 확인한다.
#
# 나침반 바늘: navrelposned의 relPosHeading은 이미 사람이 읽는 방위각과 같은
# 정의다(0=North, 시계 방향 +, NED). 그래서 gps_accuracy_gui의 CompassWidget과
# 달리 ENU->bearing 변환이 필요 없다 -- 대신 imu/heading 쪽만 ENU yaw이므로
# 그것만 역변환해서 raw 값과 비교한다.
#
# 실행:
#   ros2 run hyper_localization rtk_heading_monitor.py
#   ros2 run hyper_localization rtk_heading_monitor.py --ros-args \
#       -p navrelposned_topic:=/ublox_gps_node_rover/navrelposned
#
# 확인 절차(hyper_rtk/README.md의 "먼저 확인한다" 항목을 화면으로 보는 버전):
#   1. 정지 상태에서 STATUS가 초록(RTK FIXED, heading valid)이 되는지.
#   2. 차량을 손으로 돌려보며 나침반 바늘과 아래쪽 이력 그래프가 그럴듯하게
#      따라오는지(끊기거나 180도 튀지 않는지).
#   3. RAW(navrelposned)와 IMU(변환 결과)의 방위각 차이가 항상 ~0deg인지 --
#      벌어지면 드라이버 변환이나 remap이 잘못된 것.
# =====================================================================

import math
import signal
import sys
import threading

from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from ublox_msgs.msg import NavRELPOSNED9

from PyQt5.QtCore import Qt, QPointF, QRectF, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QApplication, QFrame, QGridLayout, QHBoxLayout, QLabel, QVBoxLayout, QWidget)

CARRIER_SOLN = {
    NavRELPOSNED9.FLAGS_CARR_SOLN_NONE: 'NONE',
    NavRELPOSNED9.FLAGS_CARR_SOLN_FLOAT: 'FLOAT',
    NavRELPOSNED9.FLAGS_CARR_SOLN_FIXED: 'FIXED',
}

COLOR_GOOD = '#3fb950'
COLOR_OK = '#d29922'
COLOR_BAD = '#f85149'
COLOR_STALE = '#6e7681'
COLOR_TEXT = '#c9d1d9'
COLOR_BG = '#0d1117'
COLOR_RAW = '#58a6ff'    # navrelposned(원본) 계열
COLOR_IMU = '#bc8cff'    # imu/heading(변환 결과) 계열

STALE_MS = 2000

# 이력 그래프에 담아 둘 구간 길이. 차를 돌려 보며 바늘이 따라오는지 눈으로
# 확인하기엔 이 정도면 충분하고, 너무 길면 오래된 값이 화면을 가득 채운다.
HISTORY_SECONDS = 20.0


def wrap_deg(deg):
    """임의의 각도를 [0, 360)으로 접는다."""
    return deg % 360.0


def angle_diff_deg(a, b):
    """두 방위각(deg) 사이의 최단 차이. 결과는 (-180, 180]."""
    d = (a - b + 180.0) % 360.0 - 180.0
    return d


def yaw_from_quaternion(q):
    """쿼터니언 -> ENU yaw [rad]. tf_transformations 의존성을 안 만들려고 직접 푼다."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def bearing_from_enu_yaw(yaw_rad):
    """ENU yaw(0=East, 반시계 +) -> 방위각(0=North, 시계 +), deg."""
    return wrap_deg(90.0 - math.degrees(yaw_rad))


class CompassWidget(QWidget):
    """RAW(navrelposned) 방위각과 IMU(imu/heading 역변환) 방위각을 겹쳐 그린다.

    둘이 일치하면 바늘이 완전히 겹쳐 보인다 -- 벌어지면 변환/remap 어딘가가
    깨졌다는 뜻. head_valid가 꺼져 있으면 RAW 바늘을 빨갛게(믿지 마라) 그린다.
    """

    def __init__(self, diameter=180):
        super().__init__()
        self._raw_bearing = None
        self._raw_valid = False
        self._carrier = None
        self._imu_bearing = None
        self.setFixedSize(diameter, diameter)

    def set_raw(self, bearing_deg, head_valid, carrier):
        self._raw_bearing = bearing_deg
        self._raw_valid = head_valid
        self._carrier = carrier
        self.update()

    def set_imu(self, bearing_deg):
        self._imu_bearing = bearing_deg
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        side = min(self.width(), self.height())
        cx, cy = self.width() / 2.0, self.height() / 2.0
        r = side / 2.0 - 14.0
        live = self._raw_bearing is not None
        ring = QColor(COLOR_TEXT if live else COLOR_STALE)

        p.setPen(QPen(ring, 2))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QRectF(cx - r, cy - r, 2 * r, 2 * r))

        # 30도마다 눈금, 90도마다 길게
        for deg in range(0, 360, 30):
            a = math.radians(deg)
            inner = r - (10 if deg % 90 == 0 else 5)
            p.setPen(QPen(ring, 2 if deg % 90 == 0 else 1))
            p.drawLine(QPointF(cx + inner * math.sin(a), cy - inner * math.cos(a)),
                       QPointF(cx + r * math.sin(a), cy - r * math.cos(a)))

        p.setFont(QFont('DejaVu Sans', 9, QFont.Bold))
        for label, deg in (('N', 0), ('E', 90), ('S', 180), ('W', 270)):
            a = math.radians(deg)
            tx = cx + (r + 8) * math.sin(a)
            ty = cy - (r + 8) * math.cos(a)
            p.setPen(QPen(QColor(COLOR_BAD if label == 'N' else
                                 (COLOR_TEXT if live else COLOR_STALE))))
            p.drawText(QRectF(tx - 10, ty - 9, 20, 18), Qt.AlignCenter, label)

        if not live:
            p.setPen(QPen(QColor(COLOR_STALE)))
            p.setFont(QFont('DejaVu Sans', 8))
            p.drawText(QRectF(cx - 50, cy - 8, 100, 16), Qt.AlignCenter, 'no navrelposned')
            return

        # IMU(변환 결과) 바늘 -- 가는 선, RAW 뒤에 깔아서 겹침을 눈으로 보게 한다
        if self._imu_bearing is not None:
            a = math.radians(self._imu_bearing)
            tip = QPointF(cx + (r - 10) * math.sin(a), cy - (r - 10) * math.cos(a))
            p.setPen(QPen(QColor(COLOR_IMU), 4))
            p.drawLine(QPointF(cx, cy), tip)

        # RAW(navrelposned) 바늘 -- 채운 삼각형. head_valid=False면 빨강으로 경고.
        needle_color = QColor(COLOR_RAW if self._raw_valid else COLOR_BAD)
        a = math.radians(self._raw_bearing)
        tip = QPointF(cx + (r - 16) * math.sin(a), cy - (r - 16) * math.cos(a))
        left = QPointF(cx + 8 * math.sin(a + math.pi * 0.5),
                       cy - 8 * math.cos(a + math.pi * 0.5))
        right = QPointF(cx + 8 * math.sin(a - math.pi * 0.5),
                        cy - 8 * math.cos(a - math.pi * 0.5))
        p.setPen(Qt.NoPen)
        p.setBrush(needle_color)
        p.drawPolygon(QPolygonF([tip, left, right]))
        tail = QPointF(cx - (r * 0.4) * math.sin(a), cy + (r * 0.4) * math.cos(a))
        p.setPen(QPen(needle_color, 2))
        p.drawLine(QPointF(cx, cy), tail)


class HeadingHistoryWidget(QWidget):
    """최근 HISTORY_SECONDS초의 방위각을 스트립차트로 그린다.

    차를 돌리며 바늘이 튀거나 끊기지 않고 매끄럽게 따라오는지, 정지 상태에서
    잡음이 얼마나 되는지를 눈으로 보기 위한 것. 360/0 경계에서 선이 화면을
    가로질러 그어지지 않도록 이전 값 기준으로 unwrap해서 그린다(표시용일 뿐,
    실제 값은 그대로 mod 360로 관리).
    """

    def __init__(self, height=110):
        super().__init__()
        self.setMinimumHeight(height)
        self._samples = deque()  # (monotonic_s, unwrapped_deg, carrier)
        self._unwrapped_last = None

    def add_sample(self, now_s, bearing_deg, carrier):
        if self._unwrapped_last is None:
            unwrapped = bearing_deg
        else:
            unwrapped = self._unwrapped_last + angle_diff_deg(bearing_deg, self._unwrapped_last % 360.0)
        self._unwrapped_last = unwrapped
        self._samples.append((now_s, unwrapped, carrier))
        while self._samples and now_s - self._samples[0][0] > HISTORY_SECONDS:
            self._samples.popleft()
        self.update()

    def clear(self):
        self._samples.clear()
        self._unwrapped_last = None
        self.update()

    def recent_stddev_deg(self, window_s=5.0):
        """최근 window_s초 구간의 표준편차(deg). 표본이 2개 미만이면 None.

        정지 상태에서 이 값이 작아야(대략 accHeading 수준) "잘 들어오는" 것.
        """
        if len(self._samples) < 2:
            return None
        latest = self._samples[-1][0]
        vals = [v for t, v, _ in self._samples if latest - t <= window_s]
        if len(vals) < 2:
            return None
        mean = sum(vals) / len(vals)
        var = sum((v - mean) ** 2 for v in vals) / len(vals)
        return math.sqrt(var)

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        p.setPen(QPen(QColor('#21262d')))
        p.setBrush(QColor(COLOR_BG))
        p.drawRect(0, 0, w - 1, h - 1)

        if len(self._samples) < 2:
            p.setPen(QPen(QColor(COLOR_STALE)))
            p.setFont(QFont('DejaVu Sans', 9))
            p.drawText(QRectF(0, 0, w, h), Qt.AlignCenter, 'waiting for heading history...')
            return

        t_max = self._samples[-1][0]
        t_min = t_max - HISTORY_SECONDS
        vals = [v for _, v, _ in self._samples]
        v_lo, v_hi = min(vals), max(vals)
        # 값이 거의 평평해도(정지 중) 그래프가 한 줄로 안 뭉개지게 최소 폭을 둔다.
        span = max(v_hi - v_lo, 4.0)
        v_lo -= span * 0.15
        v_hi += span * 0.15
        span = v_hi - v_lo

        def to_point(t, v):
            x = (t - t_min) / HISTORY_SECONDS * w
            y = h - (v - v_lo) / span * h
            return QPointF(x, y)

        carrier_color = {
            NavRELPOSNED9.FLAGS_CARR_SOLN_FIXED: COLOR_GOOD,
            NavRELPOSNED9.FLAGS_CARR_SOLN_FLOAT: COLOR_OK,
            NavRELPOSNED9.FLAGS_CARR_SOLN_NONE: COLOR_BAD,
        }
        pts = [to_point(t, v) for t, v, _ in self._samples]
        for i in range(1, len(pts)):
            _, _, carrier = self._samples[i]
            p.setPen(QPen(QColor(carrier_color.get(carrier, COLOR_STALE)), 2))
            p.drawLine(pts[i - 1], pts[i])

        p.setPen(QPen(QColor(COLOR_STALE)))
        p.setFont(QFont('DejaVu Sans Mono', 8))
        p.drawText(QRectF(4, 2, 200, 14), Qt.AlignLeft, f'{v_hi:6.1f} deg')
        p.drawText(QRectF(4, h - 16, 200, 14), Qt.AlignLeft, f'{v_lo:6.1f} deg')
        p.drawText(QRectF(w - 90, h - 16, 86, 14), Qt.AlignRight, f'-{HISTORY_SECONDS:.0f}s..now')


class RtkHeadingWindow(QWidget):
    """ROS 콜백은 rclpy 스레드에서 오므로 위젯을 직접 못 만진다.
    pyqtSignal로 넘겨 Qt 메인 스레드에서만 갱신한다."""

    relposned = pyqtSignal(object)
    imu_heading = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self.setWindowTitle('HYPER RTK Heading Monitor')
        self.setMinimumWidth(640)

        self._status = QLabel('waiting for data...')
        self._flags_detail = QLabel('')
        self._heading_val = self._big_value()
        self._acc_val = self._big_value()
        self._baseline_val = self._big_value()
        self._compass = CompassWidget()
        self._cross_check = QLabel('--')
        self._history = HeadingHistoryWidget()
        self._stability = QLabel('--')
        self._source = QLabel('')

        self._raw_bearing = None
        self._raw_valid = False
        self._carrier = None
        # 이력 그래프의 x축(경과 시간)용 단조 시계 -- main()의 QTimer가 채운다.
        self._now_s = 0.0

        root = QVBoxLayout(self)
        root.setContentsMargins(24, 18, 24, 16)
        root.setSpacing(12)

        self._status.setFont(QFont('DejaVu Sans', 16, QFont.Bold))
        self._status.setAlignment(Qt.AlignCenter)
        root.addWidget(self._status)

        self._flags_detail.setFont(QFont('DejaVu Sans Mono', 9))
        self._flags_detail.setAlignment(Qt.AlignCenter)
        self._flags_detail.setStyleSheet(f'color: {COLOR_STALE};')
        root.addWidget(self._flags_detail)

        root.addWidget(self._divider())

        vals = QGridLayout()
        vals.setHorizontalSpacing(24)
        vals.addWidget(self._caption('relPosHeading (bearing)'), 0, 0)
        vals.addWidget(self._caption('accHeading'), 0, 1)
        vals.addWidget(self._caption('baseline length'), 0, 2)
        vals.addWidget(self._heading_val, 1, 0)
        vals.addWidget(self._acc_val, 1, 1)
        vals.addWidget(self._baseline_val, 1, 2)
        root.addLayout(vals)

        root.addWidget(self._divider())

        bottom = QHBoxLayout()
        bottom.setSpacing(22)

        compass_col = QVBoxLayout()
        compass_col.setSpacing(4)
        compass_col.addWidget(self._compass, alignment=Qt.AlignCenter)
        legend = QLabel(f'<span style="color:{COLOR_RAW}">▲ raw (navrelposned)</span>'
                         f'&nbsp;&nbsp;<span style="color:{COLOR_IMU}">| imu/heading</span>')
        legend.setFont(QFont('DejaVu Sans', 8))
        legend.setAlignment(Qt.AlignCenter)
        compass_col.addWidget(legend)
        bottom.addLayout(compass_col)

        right_col = QVBoxLayout()
        right_col.setSpacing(8)
        right_col.addWidget(self._caption('raw vs imu/heading cross-check'))
        self._cross_check.setFont(QFont('DejaVu Sans Mono', 12, QFont.Bold))
        self._cross_check.setAlignment(Qt.AlignCenter)
        right_col.addWidget(self._cross_check)
        right_col.addWidget(self._caption(f'stddev, last 5s (stationary should be small)'))
        self._stability.setFont(QFont('DejaVu Sans Mono', 12, QFont.Bold))
        self._stability.setAlignment(Qt.AlignCenter)
        right_col.addWidget(self._stability)
        right_col.addStretch(1)
        bottom.addLayout(right_col, 1)
        root.addLayout(bottom)

        root.addWidget(self._caption(f'heading history (last {HISTORY_SECONDS:.0f}s) -- '
                                      'green=RTK FIXED, yellow=FLOAT, red=NONE/invalid'))
        root.addWidget(self._history)

        self._source.setFont(QFont('DejaVu Sans Mono', 8))
        self._source.setAlignment(Qt.AlignCenter)
        self._source.setStyleSheet(f'color: {COLOR_STALE};')
        root.addWidget(self._source)

        self.setStyleSheet(f'background: {COLOR_BG}; color: {COLOR_TEXT};')

        self.relposned.connect(self._render_relposned)
        self.imu_heading.connect(self._render_imu_heading)

        self._imu_bearing = None

        self._timers = {
            'relposned': self._stale_timer(self._relposned_stale),
            'imu': self._stale_timer(self._imu_stale),
        }

    # ---------- 위젯 헬퍼 ----------
    def _caption(self, text):
        label = QLabel(text)
        label.setFont(QFont('DejaVu Sans', 9))
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet(f'color: {COLOR_STALE};')
        return label

    def _big_value(self):
        label = QLabel('--')
        label.setFont(QFont('DejaVu Sans', 22, QFont.Bold))
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet(f'color: {COLOR_STALE};')
        return label

    def _divider(self):
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet('color: #21262d; background: #21262d; max-height: 1px;')
        return line

    def _stale_timer(self, slot):
        timer = QTimer(self)
        timer.timeout.connect(slot)
        timer.setInterval(STALE_MS)
        timer.start()
        return timer

    def set_topics(self, relposned_topic, imu_topic):
        self._source.setText(f'{relposned_topic}   |   {imu_topic}')

    def set_now(self, now_s):
        self._now_s = now_s

    # ---------- stale ----------
    def _relposned_stale(self):
        self._status.setText('NO DATA (stale) -- navrelposned not arriving')
        self._status.setStyleSheet(f'color: {COLOR_STALE};')
        self._flags_detail.setText('')
        self._heading_val.setStyleSheet(f'color: {COLOR_STALE};')
        self._acc_val.setStyleSheet(f'color: {COLOR_STALE};')
        self._baseline_val.setStyleSheet(f'color: {COLOR_STALE};')
        self._raw_bearing = None
        self._compass.set_raw(None, False, None)
        self._history.clear()
        self._update_cross_check()

    def _imu_stale(self):
        self._imu_bearing = None
        self._compass.set_imu(None)
        self._update_cross_check()

    # ---------- 렌더 ----------
    def _render_relposned(self, msg):
        self._timers['relposned'].start()

        fix_ok = bool(msg.flags & NavRELPOSNED9.FLAGS_GNSS_FIX_OK)
        diff_soln = bool(msg.flags & NavRELPOSNED9.FLAGS_DIFF_SOLN)
        rel_pos_valid = bool(msg.flags & NavRELPOSNED9.FLAGS_REL_POS_VALID)
        head_valid = bool(msg.flags & NavRELPOSNED9.FLAGS_REL_POS_HEAD_VALID)
        is_moving = bool(msg.flags & NavRELPOSNED9.FLAGS_IS_MOVING)
        carrier = msg.flags & NavRELPOSNED9.FLAGS_CARR_SOLN_MASK
        carrier_name = CARRIER_SOLN.get(carrier, f'? ({carrier})')

        bearing = wrap_deg(msg.rel_pos_heading * 1e-5)
        acc_deg = msg.acc_heading * 1e-5
        baseline_m = (msg.rel_pos_length + msg.rel_pos_hp_length * 1e-2) / 100.0

        self._raw_bearing = bearing
        self._raw_valid = head_valid
        self._carrier = carrier
        self._compass.set_raw(bearing, head_valid, carrier)
        self._history.add_sample(self._now_s, bearing, carrier)

        self._heading_val.setText(f'{bearing:6.1f}°')
        self._acc_val.setText(f'±{acc_deg:.2f}°')
        self._baseline_val.setText(f'{baseline_m:.3f} m')

        if not head_valid:
            self._heading_val.setStyleSheet(f'color: {COLOR_BAD};')
        elif carrier == NavRELPOSNED9.FLAGS_CARR_SOLN_FIXED:
            self._heading_val.setStyleSheet(f'color: {COLOR_GOOD};')
        else:
            self._heading_val.setStyleSheet(f'color: {COLOR_OK};')
        self._acc_val.setStyleSheet(
            f'color: {COLOR_GOOD if acc_deg <= 1.0 else (COLOR_OK if acc_deg <= 5.0 else COLOR_BAD)};')
        self._baseline_val.setStyleSheet(f'color: {COLOR_TEXT};')

        if not fix_ok:
            self._status.setText('NO GNSS FIX -- heading meaningless')
            self._status.setStyleSheet(f'color: {COLOR_BAD};')
        elif not rel_pos_valid or not head_valid:
            self._status.setText(f'HEADING NOT VALID YET  ({carrier_name} solution)')
            self._status.setStyleSheet(f'color: {COLOR_BAD};')
        elif carrier == NavRELPOSNED9.FLAGS_CARR_SOLN_FIXED:
            self._status.setText('RTK FIXED -- heading valid')
            self._status.setStyleSheet(f'color: {COLOR_GOOD};')
        elif carrier == NavRELPOSNED9.FLAGS_CARR_SOLN_FLOAT:
            self._status.setText('RTK FLOAT -- heading valid but coarse')
            self._status.setStyleSheet(f'color: {COLOR_OK};')
        else:
            self._status.setText(f'heading valid ({carrier_name} solution)')
            self._status.setStyleSheet(f'color: {COLOR_OK};')

        self._flags_detail.setText(
            f'fixOK={int(fix_ok)}  diffSoln={int(diff_soln)}  relPosValid={int(rel_pos_valid)}  '
            f'headValid={int(head_valid)}  isMoving={int(is_moving)}  carrSoln={carrier_name}')

        stddev = self._history.recent_stddev_deg()
        if stddev is None:
            self._stability.setText('--')
            self._stability.setStyleSheet(f'color: {COLOR_STALE};')
        else:
            self._stability.setText(f'{stddev:.2f}°')
            self._stability.setStyleSheet(
                f'color: {COLOR_GOOD if stddev <= 1.0 else (COLOR_OK if stddev <= 5.0 else COLOR_BAD)};')

        self._update_cross_check()

    def _render_imu_heading(self, msg):
        self._timers['imu'].start()
        yaw = yaw_from_quaternion(msg.orientation)
        self._imu_bearing = bearing_from_enu_yaw(yaw)
        self._compass.set_imu(self._imu_bearing)
        self._update_cross_check()

    def _update_cross_check(self):
        if self._raw_bearing is None or self._imu_bearing is None:
            self._cross_check.setText('--')
            self._cross_check.setStyleSheet(f'color: {COLOR_STALE};')
            return
        diff = angle_diff_deg(self._raw_bearing, self._imu_bearing)
        self._cross_check.setText(
            f'raw {self._raw_bearing:6.1f}°  imu {self._imu_bearing:6.1f}°  '
            f'Δ{diff:+5.1f}°')
        # 0.1도 넘게 벌어지면 반올림 오차 수준을 넘어선 것 -- remap/변환을 의심.
        self._cross_check.setStyleSheet(
            f'color: {COLOR_GOOD if abs(diff) <= 0.1 else COLOR_BAD};')


class RtkHeadingNode(Node):
    def __init__(self, window):
        super().__init__('rtk_heading_monitor')

        def param(name, default):
            return self.declare_parameter(
                name, default).get_parameter_value().string_value

        self.relposned_topic = param(
            'navrelposned_topic', '/ublox_gps_node_rover/navrelposned')
        self.imu_topic = param('imu_heading_topic', 'imu/heading')

        # ublox_gps는 create_publisher<NavRELPOSNED9>("navrelposned", 1)/
        # create_publisher<Imu>("navheading", 1) -- 둘 다 기본 QoS(RELIABLE/
        # KEEP_LAST)라 여기서도 기본값으로 맞춘다.
        self.create_subscription(
            NavRELPOSNED9, self.relposned_topic, lambda m: window.relposned.emit(m), 10)
        self.create_subscription(
            Imu, self.imu_topic, lambda m: window.imu_heading.emit(m), 10)

        self.get_logger().info(
            f'RTK heading monitor: {self.relposned_topic} | {self.imu_topic}')


def main():
    rclpy.init(args=sys.argv)
    app = QApplication(sys.argv)
    # Qt가 SIGINT를 삼키므로 Ctrl-C가 먹도록 기본 핸들러로 되돌린다.
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    window = RtkHeadingWindow()
    node = RtkHeadingNode(window)
    window.set_topics(node.relposned_topic, node.imu_topic)
    window.show()

    # 이력 그래프의 시간축은 ROS 시계가 아니라 단조 시계로 충분하다(경과 시간만
    # 필요). navrelposned가 올 때마다 그 시점의 monotonic 시각을 붙여 넣도록
    # 짧은 주기로 갱신해 둔다.
    import time as _time

    now_timer = QTimer(window)
    now_timer.timeout.connect(lambda: window.set_now(_time.monotonic()))
    now_timer.setInterval(50)
    now_timer.start()
    window.set_now(_time.monotonic())

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
