#!/usr/bin/env python3
# =====================================================================
# HYPER RTK 모니터 -- 절대 위치 + 절대 방위를 한 창에서 본다
#
# 예전에는 창이 둘이었다(gps_accuracy_gui = 위치, rtk_heading_monitor = 방위).
# 나침반 바늘이 어느 센서에서 온 건지가 창마다 달라서 헷갈렸기 때문에 하나로 합쳤다.
#
# 색 규칙 -- 이 창에서 색은 곧 출처다
# ------------------------------------
#   파랑(#58a6ff) = GNSS 원본 측정값   (/odometry/gps, navrelposned)
#   보라(#bc8cff) = EKF 융합 결과      (/odometry/filtered_map)
# 위치든 방위든 같은 규칙이라, 파랑과 보라가 벌어져 있으면 그게 곧 필터가
# 원본에서 멀어진 정도다.
#
# 나침반 바늘이 IMU에서 오지 "않는다"
# ------------------------------------
# 이 차의 절대 방위는 오직 듀얼 안테나 baseline(base/rover 두 ZED-F9P의 상대 위치)
# 에서만 나온다. EBIMU-9DOFV5의 지자기 yaw는 dual_ekf_navsat.yaml에서 두 필터 모두
# imu0_config의 yaw(인덱스 5)가 false라 EKF에 아예 들어가지 않는다 -- 들어가는 건
# imu1(imu/heading), 즉 RTK baseline 헤딩뿐이다. 그래서:
#
#   채운 삼각형 바늘 = RTK baseline 원본 (navrelposned의 relPosHeading). 진실.
#   가는 선 바늘     = EKF가 낸 yaw (/odometry/filtered_map). 차가 실제로 믿고 달리는 값.
#
# IMU(/imu)는 이 창에서 방위에 전혀 기여하지 않는다. 아래쪽 "IMU link" 줄은 순전히
# 센서가 살아 있는지(토픽이 흐르는지)만 본다 -- roll/pitch와 각속도용이다.
#
# 왜 navrelposned와 imu/heading을 둘 다 보는가
# ---------------------------------------------
# ublox_gps의 hp_pos_rec_product.cpp는 FLAGS_REL_POS_HEAD_VALID가 꺼져 있어도
# imu/heading을 계속 publish한다 -- orientation_covariance[8]을 1000 rad^2("믿지 마라")로
# 채울 뿐이다. 그래서 imu/heading만 보면 fix를 못 잡았을 때도 그럴듯한 숫자가 계속
# 흐르는 것처럼 보인다. 실제 유효성은 navrelposned의 flags에만 있다. 두 값의 차이
# (Δ)는 드라이버의 NED->ENU 변환이 제대로 됐는지 확인하는 용도라 항상 ~0이어야 한다.
#
# 토픽
# ----
#   /ublox_gps_node_base/navpvt         (ublox_msgs/NavPVT)        hAcc/vAcc, fix, RTK, 위성수
#   /ublox_gps_node_rover/navrelposned  (ublox_msgs/NavRELPOSNED9) 방위 원본 + 유효성 flags
#   /imu/heading                        (sensor_msgs/Imu)          위를 ENU yaw로 변환한 결과
#   /odometry/gps                       (nav_msgs/Odometry)        GPS만으로 푼 map x/y
#   /odometry/filtered_map              (nav_msgs/Odometry)        EKF 융합 map x/y + yaw
#   /imu                                (sensor_msgs/Imu)          EBIMU 링크 생존 확인용
#
# navpvt가 base, navrelposned가 rover인 이유: hAcc/vAcc/RTK 비트는 절대 위치(base)의
# 정확도고, 헤딩 신뢰도(accHeading)는 moving-base인 rover 쪽에 있다.
#
# 실행:
#   ros2 run hyper_localization gps_accuracy_gui.py
#   ros2 run hyper_localization gps_accuracy_gui.py --ros-args \
#       -p navrelposned_topic:=/other/navrelposned
#
# 실차 확인 절차:
#   1. 정지 상태에서 HEADING 쪽 STATUS가 초록(RTK FIXED, heading valid)이 되는지.
#   2. 차를 손으로 돌려보며 삼각형 바늘과 이력 그래프가 매끄럽게 따라오는지
#      (끊기거나 180도 튀지 않는지).
#   3. raw vs imu/heading Δ가 항상 ~0°인지 -- 벌어지면 드라이버 변환이나 remap 문제.
#   4. 가는 보라 바늘(EKF)이 삼각형을 따라오는지 -- 계속 벌어지면 EKF가 헤딩을
#      제대로 못 먹고 있는 것(dual_ekf_navsat.yaml의 imu1 확인).
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
from ublox_msgs.msg import NavPVT, NavRELPOSNED9

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

CARRIER_SOLN = {
    NavRELPOSNED9.FLAGS_CARR_SOLN_NONE: 'NONE',
    NavRELPOSNED9.FLAGS_CARR_SOLN_FLOAT: 'FLOAT',
    NavRELPOSNED9.FLAGS_CARR_SOLN_FIXED: 'FIXED',
}

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
COLOR_LINE = '#21262d'
# 색 = 출처. 파랑은 GNSS 원본, 보라는 EKF 융합 결과 -- 위치와 방위 양쪽에 같은 규칙.
COLOR_RAW = '#58a6ff'
COLOR_EKF = '#bc8cff'

STALE_MS = 2000

# IMU 링크는 GPS보다 훨씬 빠르게(수십~수백 Hz) 도는 토픽이라 잠깐만 끊겨도 바로
# 티가 난다. 1초면 재연결 대기(hyper_ebimu의 reconnect_wait_seconds 기본 2초)보다
# 짧아서 끊김을 곧바로 잡아내면서도 시리얼 지터에 오탐하지 않는다.
IMU_STALE_S = 1.0
IMU_POLL_MS = 500
# Hz는 여러 폴링 구간에 걸쳐 평균낸다. 시리얼 버퍼에 몰려 있던 샘플이 한꺼번에
# 올라오면 500ms 창 하나만으로는 같은 100Hz 센서가 27Hz -> 144Hz로 튄다.
IMU_RATE_WINDOW = 4

# 이력 그래프에 담아 둘 구간 길이. 차를 돌려 보며 바늘이 따라오는지 눈으로
# 확인하기엔 이 정도면 충분하고, 너무 길면 오래된 값이 화면을 가득 채운다.
HISTORY_SECONDS = 20.0


def format_accuracy(metres):
    """hAcc/vAcc를 창 폭 안에 들어가는 길이로 찍는다.

    fix가 없을 때 u-blox는 hAcc로 수십 km(수천만 mm)를 내보내므로 항상 .3f로
    찍으면 큰 글씨가 창 밖으로 잘려 나간다. 크기에 따라 자릿수를 줄인다.
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


def wrap_deg(deg):
    """임의의 각도를 [0, 360)으로 접는다."""
    return deg % 360.0


def angle_diff_deg(a, b):
    """두 방위각(deg) 사이의 최단 차이. 결과는 (-180, 180]."""
    return (a - b + 180.0) % 360.0 - 180.0


def yaw_from_quaternion(q):
    """쿼터니언 -> ENU yaw [rad]. tf_transformations 의존성을 안 만들려고 직접 푼다."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def bearing_from_enu_yaw(yaw_rad):
    """ENU yaw(0=East, 반시계 +) -> 방위각(0=North, 시계 +), deg.

    navrelposned의 relPosHeading은 이미 방위각(NED, 0=North, 시계 +)이라 변환이
    필요 없다. 변환이 필요한 건 ENU로 나오는 쪽(imu/heading, EKF yaw)뿐이다.
    """
    return wrap_deg(90.0 - math.degrees(yaw_rad))


def unwrap_deg(prev, bearing):
    """이력 그래프용: 360/0 경계에서 선이 화면을 가로지르지 않도록 이어 붙인다.

    표시용일 뿐 실제 값은 그대로 [0,360)으로 관리한다.
    """
    if prev is None:
        return bearing
    return prev + angle_diff_deg(bearing, prev % 360.0)


class CompassWidget(QWidget):
    """RTK baseline 방위(원본)와 EKF yaw(융합 결과)를 겹쳐 그린다.

    채운 삼각형 = navrelposned의 relPosHeading. 이 차에서 절대 방위의 유일한
    출처다. head_valid가 꺼져 있으면 빨갛게(믿지 마라) 그린다.
    가는 선 = /odometry/filtered_map의 yaw. 차가 실제로 믿고 달리는 값이라,
    삼각형에서 벌어지면 EKF가 헤딩을 제대로 못 먹고 있다는 뜻.
    """

    def __init__(self, diameter=190):
        super().__init__()
        self._raw_bearing = None
        self._raw_valid = False
        self._ekf_bearing = None
        self.setFixedSize(diameter, diameter)

    def set_raw(self, bearing_deg, head_valid):
        self._raw_bearing = bearing_deg
        self._raw_valid = head_valid
        self.update()

    def set_ekf(self, bearing_deg):
        self._ekf_bearing = bearing_deg
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        side = min(self.width(), self.height())
        cx, cy = self.width() / 2.0, self.height() / 2.0
        r = side / 2.0 - 14.0
        live = self._raw_bearing is not None or self._ekf_bearing is not None
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
            p.drawText(QRectF(cx - 60, cy - 8, 120, 16), Qt.AlignCenter,
                       'no heading data')
            return

        # EKF 바늘을 먼저(뒤에) 깔아서 RTK 삼각형과의 겹침이 눈에 보이게 한다.
        if self._ekf_bearing is not None:
            a = math.radians(self._ekf_bearing)
            p.setPen(QPen(QColor(COLOR_EKF), 4))
            p.drawLine(QPointF(cx, cy),
                       QPointF(cx + (r - 10) * math.sin(a),
                               cy - (r - 10) * math.cos(a)))

        if self._raw_bearing is not None:
            needle = QColor(COLOR_RAW if self._raw_valid else COLOR_BAD)
            a = math.radians(self._raw_bearing)
            tip = QPointF(cx + (r - 16) * math.sin(a), cy - (r - 16) * math.cos(a))
            left = QPointF(cx + 8 * math.sin(a + math.pi * 0.5),
                           cy - 8 * math.cos(a + math.pi * 0.5))
            right = QPointF(cx + 8 * math.sin(a - math.pi * 0.5),
                            cy - 8 * math.cos(a - math.pi * 0.5))
            p.setPen(Qt.NoPen)
            p.setBrush(needle)
            p.drawPolygon(QPolygonF([tip, left, right]))
            tail = QPointF(cx - (r * 0.4) * math.sin(a), cy + (r * 0.4) * math.cos(a))
            p.setPen(QPen(needle, 2))
            p.drawLine(QPointF(cx, cy), tail)


class HeadingHistoryWidget(QWidget):
    """최근 HISTORY_SECONDS초의 RTK 방위와 EKF yaw를 겹쳐 스트립차트로 그린다.

    차를 돌리며 값이 튀거나 끊기지 않고 매끄럽게 따라오는지, 정지 상태에서
    잡음이 얼마나 되는지, 그리고 EKF가 원본을 얼마나 잘 따라오는지를 본다.
    RTK 선은 carrier solution에 따라 색이 바뀐다(초록=FIXED, 노랑=FLOAT, 빨강=NONE).
    """

    def __init__(self, height=120):
        super().__init__()
        self.setMinimumHeight(height)
        self._raw = deque()      # (t, unwrapped_deg, carrier)
        self._ekf = deque()      # (t, unwrapped_deg)
        self._raw_last = None
        self._ekf_last = None

    def add_raw(self, bearing_deg, carrier):
        now = time.monotonic()
        self._raw_last = unwrap_deg(self._raw_last, bearing_deg)
        self._raw.append((now, self._raw_last, carrier))
        self._trim(now)
        self.update()

    def add_ekf(self, bearing_deg):
        now = time.monotonic()
        # 첫 EKF 표본은 RTK의 현재 unwrap 값 근처에 앉힌다. 그래야 두 선이 같은
        # 연속 좌표계에 놓여 세로로 비교된다(안 그러면 360도 어긋난 채 그려진다).
        seed = self._ekf_last if self._ekf_last is not None else self._raw_last
        self._ekf_last = unwrap_deg(seed, bearing_deg)
        self._ekf.append((now, self._ekf_last))
        self._trim(now)
        self.update()

    def _trim(self, now):
        for series in (self._raw, self._ekf):
            while series and now - series[0][0] > HISTORY_SECONDS:
                series.popleft()

    def clear_raw(self):
        self._raw.clear()
        self._raw_last = None
        self.update()

    def clear_ekf(self):
        self._ekf.clear()
        self._ekf_last = None
        self.update()

    def recent_stddev_deg(self, window_s=5.0):
        """최근 window_s초 RTK 방위의 표준편차(deg). 표본이 2개 미만이면 None.

        정지 상태에서 이 값이 작아야(대략 accHeading 수준) "잘 들어오는" 것.
        """
        if len(self._raw) < 2:
            return None
        latest = self._raw[-1][0]
        vals = [v for t, v, _ in self._raw if latest - t <= window_s]
        if len(vals) < 2:
            return None
        mean = sum(vals) / len(vals)
        var = sum((v - mean) ** 2 for v in vals) / len(vals)
        return math.sqrt(var)

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        p.setPen(QPen(QColor(COLOR_LINE)))
        p.setBrush(QColor(COLOR_BG))
        p.drawRect(0, 0, w - 1, h - 1)

        if len(self._raw) < 2 and len(self._ekf) < 2:
            p.setPen(QPen(QColor(COLOR_STALE)))
            p.setFont(QFont('DejaVu Sans', 9))
            p.drawText(QRectF(0, 0, w, h), Qt.AlignCenter,
                       'waiting for heading history...')
            return

        t_max = max([s[0] for s in self._raw] + [s[0] for s in self._ekf])
        t_min = t_max - HISTORY_SECONDS
        vals = [s[1] for s in self._raw] + [s[1] for s in self._ekf]
        v_lo, v_hi = min(vals), max(vals)
        # 값이 거의 평평해도(정지 중) 그래프가 한 줄로 안 뭉개지게 최소 폭을 둔다.
        span = max(v_hi - v_lo, 4.0)
        v_lo -= span * 0.15
        v_hi += span * 0.15
        span = v_hi - v_lo

        def to_point(t, v):
            return QPointF((t - t_min) / HISTORY_SECONDS * w,
                           h - (v - v_lo) / span * h)

        # EKF를 먼저 깔고 RTK를 위에 -- 나침반과 같은 순서라 눈이 헷갈리지 않는다.
        if len(self._ekf) >= 2:
            p.setPen(QPen(QColor(COLOR_EKF), 1))
            pts = [to_point(t, v) for t, v in self._ekf]
            for i in range(1, len(pts)):
                p.drawLine(pts[i - 1], pts[i])

        carrier_color = {
            NavRELPOSNED9.FLAGS_CARR_SOLN_FIXED: COLOR_GOOD,
            NavRELPOSNED9.FLAGS_CARR_SOLN_FLOAT: COLOR_OK,
            NavRELPOSNED9.FLAGS_CARR_SOLN_NONE: COLOR_BAD,
        }
        if len(self._raw) >= 2:
            pts = [to_point(t, v) for t, v, _ in self._raw]
            for i in range(1, len(pts)):
                carrier = self._raw[i][2]
                p.setPen(QPen(QColor(carrier_color.get(carrier, COLOR_STALE)), 2))
                p.drawLine(pts[i - 1], pts[i])

        p.setPen(QPen(QColor(COLOR_STALE)))
        p.setFont(QFont('DejaVu Sans Mono', 8))
        p.drawText(QRectF(4, 2, 200, 14), Qt.AlignLeft, f'{v_hi:6.1f}°')
        p.drawText(QRectF(4, h - 16, 200, 14), Qt.AlignLeft, f'{v_lo:6.1f}°')
        p.drawText(QRectF(w - 96, h - 16, 92, 14), Qt.AlignRight,
                   f'-{HISTORY_SECONDS:.0f}s..now')


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


class RtkMonitorWindow(QWidget):
    """ROS 콜백은 rclpy 스레드에서 오므로 위젯을 직접 못 만진다.
    pyqtSignal로 넘겨 Qt 메인 스레드에서만 갱신한다.

    왼쪽 열이 위치(어디에 있나), 오른쪽 열이 방위(어디를 보고 있나)다.
    """

    navpvt = pyqtSignal(object)
    gps_odom = pyqtSignal(object)
    ekf_odom = pyqtSignal(object)
    relposned = pyqtSignal(object)
    imu_heading = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self.setWindowTitle('HYPER RTK Monitor -- position & heading')
        self.setMinimumWidth(1000)

        # ----- 상태 -----
        self._gps_pos = None
        self._ekf_pos = None
        self._raw_bearing = None
        self._raw_valid = False
        self._imu_heading_bearing = None
        self._ekf_bearing = None
        self._imu_monitor = None

        root = QVBoxLayout(self)
        root.setContentsMargins(22, 16, 22, 14)
        root.setSpacing(10)

        columns = QHBoxLayout()
        columns.setSpacing(26)
        columns.addLayout(self._build_position_column(), 1)
        columns.addWidget(self._vdivider())
        columns.addLayout(self._build_heading_column(), 1)
        root.addLayout(columns)

        root.addWidget(self._divider())

        self._history = HeadingHistoryWidget()
        root.addWidget(self._caption(
            f'heading history, last {HISTORY_SECONDS:.0f}s  --  thick line = RTK baseline '
            '(green FIXED / yellow FLOAT / red NONE),  thin line = EKF yaw'))
        root.addWidget(self._history)

        root.addWidget(self._divider())

        imu_row = QHBoxLayout()
        imu_row.setSpacing(10)
        imu_caption = self._caption(
            'IMU link  E2BOX EBIMU-9DOFV5 (USB-UART)  --  roll/pitch + rates only, '
            'contributes no heading')
        imu_caption.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        imu_row.addWidget(imu_caption)
        self._imu_link = QLabel('--')
        self._imu_link.setFont(QFont('DejaVu Sans Mono', 11, QFont.Bold))
        self._imu_link.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self._imu_link.setStyleSheet(f'color: {COLOR_STALE};')
        imu_row.addWidget(self._imu_link, 1)
        root.addLayout(imu_row)

        self._source = QLabel('')
        self._source.setFont(QFont('DejaVu Sans Mono', 8))
        self._source.setAlignment(Qt.AlignCenter)
        self._source.setStyleSheet(f'color: {COLOR_STALE};')
        root.addWidget(self._source)

        self.setStyleSheet(f'background: {COLOR_BG}; color: {COLOR_TEXT};')

        self.navpvt.connect(self._render_navpvt)
        self.gps_odom.connect(self._render_gps_odom)
        self.ekf_odom.connect(self._render_ekf_odom)
        self.relposned.connect(self._render_relposned)
        self.imu_heading.connect(self._render_imu_heading)

        # 수신이 끊겨도 마지막 값이 그대로 남아 있으면 오해하기 쉬우므로,
        # 토픽별로 STALE_MS 넘게 조용하면 회색으로 떨어뜨린다.
        self._timers = {
            'navpvt': self._stale_timer(self._navpvt_stale),
            'gps': self._stale_timer(self._gps_stale),
            'ekf': self._stale_timer(self._ekf_stale),
            'relposned': self._stale_timer(self._relposned_stale),
            'imu_heading': self._stale_timer(self._imu_heading_stale),
        }

        self._imu_timer = QTimer(self)
        self._imu_timer.timeout.connect(self._render_imu_link)
        self._imu_timer.setInterval(IMU_POLL_MS)
        self._imu_timer.start()

    # ---------- 열 구성 ----------
    def _build_position_column(self):
        col = QVBoxLayout()
        col.setSpacing(8)
        col.addWidget(self._section('POSITION   --   base antenna, absolute fix'))

        self._h_acc = self._big_value(26)
        self._v_acc = self._big_value(26)
        acc = QGridLayout()
        acc.setHorizontalSpacing(24)
        acc.addWidget(self._caption('Horizontal (hAcc)'), 0, 0)
        acc.addWidget(self._caption('Vertical (vAcc)'), 0, 1)
        acc.addWidget(self._h_acc, 1, 0)
        acc.addWidget(self._v_acc, 1, 1)
        col.addLayout(acc)

        self._fix_status = QLabel('waiting for data...')
        self._fix_status.setFont(QFont('DejaVu Sans', 14, QFont.Bold))
        self._fix_status.setAlignment(Qt.AlignCenter)
        col.addWidget(self._fix_status)

        self._fix_detail = QLabel('')
        self._fix_detail.setFont(QFont('DejaVu Sans Mono', 9))
        self._fix_detail.setAlignment(Qt.AlignCenter)
        self._fix_detail.setStyleSheet(f'color: {COLOR_STALE};')
        col.addWidget(self._fix_detail)

        col.addWidget(self._divider())

        self._gps_xy = QLabel('--')
        self._ekf_xy = QLabel('--')
        self._delta = QLabel('--')
        xy = QGridLayout()
        xy.setVerticalSpacing(4)
        for row, (cap, widget, colour) in enumerate((
                ('GPS only   /odometry/gps', self._gps_xy, COLOR_RAW),
                ('EKF fused  /odometry/filtered_map', self._ekf_xy, COLOR_EKF),
                ('separation  |EKF - GPS|', self._delta, COLOR_TEXT))):
            label = self._caption(cap)
            label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            widget.setFont(QFont('DejaVu Sans Mono', 13, QFont.Bold))
            widget.setStyleSheet(f'color: {colour};')
            xy.addWidget(label, row * 2, 0)
            xy.addWidget(widget, row * 2 + 1, 0)
        col.addLayout(xy)
        col.addStretch(1)
        return col

    def _build_heading_column(self):
        col = QVBoxLayout()
        col.setSpacing(8)
        col.addWidget(self._section(
            'HEADING   --   dual-antenna baseline, the only absolute yaw'))

        self._head_status = QLabel('waiting for data...')
        self._head_status.setFont(QFont('DejaVu Sans', 14, QFont.Bold))
        self._head_status.setAlignment(Qt.AlignCenter)
        col.addWidget(self._head_status)

        self._flags_detail = QLabel('')
        self._flags_detail.setFont(QFont('DejaVu Sans Mono', 9))
        self._flags_detail.setAlignment(Qt.AlignCenter)
        self._flags_detail.setStyleSheet(f'color: {COLOR_STALE};')
        col.addWidget(self._flags_detail)

        self._compass = CompassWidget()
        col.addWidget(self._compass, alignment=Qt.AlignCenter)

        legend = QLabel(
            f'<span style="color:{COLOR_RAW}">&#9650; RTK baseline &mdash; navrelposned</span>'
            f'&nbsp;&nbsp;&nbsp;'
            f'<span style="color:{COLOR_EKF}">&#9474; EKF yaw &mdash; /odometry/filtered_map</span>')
        legend.setFont(QFont('DejaVu Sans', 8))
        legend.setAlignment(Qt.AlignCenter)
        col.addWidget(legend)

        self._heading_val = self._big_value(20)
        self._acc_val = self._big_value(20)
        self._baseline_val = self._big_value(20)
        vals = QGridLayout()
        vals.setHorizontalSpacing(18)
        vals.addWidget(self._caption('relPosHeading'), 0, 0)
        vals.addWidget(self._caption('accHeading'), 0, 1)
        vals.addWidget(self._caption('baseline'), 0, 2)
        vals.addWidget(self._heading_val, 1, 0)
        vals.addWidget(self._acc_val, 1, 1)
        vals.addWidget(self._baseline_val, 1, 2)
        col.addLayout(vals)

        checks = QGridLayout()
        checks.setVerticalSpacing(3)
        self._cross_check = self._mono_value()
        self._ekf_track = self._mono_value()
        self._stability = self._mono_value()
        for row, (cap, widget) in enumerate((
                ('driver conversion  raw vs /imu/heading  (should be ~0)',
                 self._cross_check),
                ('filter tracking  raw vs EKF yaw', self._ekf_track),
                ('stddev, last 5s  (small when stationary)', self._stability))):
            checks.addWidget(self._caption(cap), row * 2, 0)
            checks.addWidget(widget, row * 2 + 1, 0)
        col.addLayout(checks)
        col.addStretch(1)
        return col

    # ---------- 위젯 헬퍼 ----------
    def _caption(self, text):
        label = QLabel(text)
        label.setFont(QFont('DejaVu Sans', 9))
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet(f'color: {COLOR_STALE};')
        return label

    def _section(self, text):
        label = QLabel(text)
        label.setFont(QFont('DejaVu Sans', 10, QFont.Bold))
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet(f'color: {COLOR_TEXT};')
        return label

    def _big_value(self, points):
        label = QLabel('--')
        label.setFont(QFont('DejaVu Sans', points, QFont.Bold))
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet(f'color: {COLOR_STALE};')
        return label

    def _mono_value(self):
        label = QLabel('--')
        label.setFont(QFont('DejaVu Sans Mono', 11, QFont.Bold))
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet(f'color: {COLOR_STALE};')
        return label

    def _divider(self):
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet(
            f'color: {COLOR_LINE}; background: {COLOR_LINE}; max-height: 1px;')
        return line

    def _vdivider(self):
        line = QFrame()
        line.setFrameShape(QFrame.VLine)
        line.setStyleSheet(
            f'color: {COLOR_LINE}; background: {COLOR_LINE}; max-width: 1px;')
        return line

    def _stale_timer(self, slot):
        timer = QTimer(self)
        timer.timeout.connect(slot)
        timer.setInterval(STALE_MS)
        timer.start()
        return timer

    def set_topics(self, navpvt, relposned, imu_heading, gps, ekf, imu):
        self._source.setText(
            f'{navpvt}   |   {relposned}   |   {imu_heading}\n'
            f'{gps}   |   {ekf}   |   {imu}')

    def set_imu_monitor(self, monitor):
        self._imu_monitor = monitor

    # ---------- stale ----------
    def _navpvt_stale(self):
        self._h_acc.setStyleSheet(f'color: {COLOR_STALE};')
        self._v_acc.setStyleSheet(f'color: {COLOR_STALE};')
        self._fix_status.setText('NO DATA (stale)')
        self._fix_status.setStyleSheet(f'color: {COLOR_STALE};')

    def _gps_stale(self):
        self._gps_pos = None
        self._gps_xy.setText('no fix / no data')
        self._gps_xy.setStyleSheet(f'color: {COLOR_STALE};')
        self._update_separation()

    def _ekf_stale(self):
        self._ekf_pos = None
        self._ekf_xy.setText('no data')
        self._ekf_xy.setStyleSheet(f'color: {COLOR_STALE};')
        self._ekf_bearing = None
        self._compass.set_ekf(None)
        self._history.clear_ekf()
        self._update_separation()
        self._update_ekf_track()

    def _relposned_stale(self):
        self._head_status.setText('NO DATA (stale) -- navrelposned not arriving')
        self._head_status.setStyleSheet(f'color: {COLOR_STALE};')
        self._flags_detail.setText('')
        for widget in (self._heading_val, self._acc_val, self._baseline_val):
            widget.setStyleSheet(f'color: {COLOR_STALE};')
        self._raw_bearing = None
        self._raw_valid = False
        self._compass.set_raw(None, False)
        self._history.clear_raw()
        self._update_cross_check()
        self._update_ekf_track()
        self._update_stability()

    def _imu_heading_stale(self):
        self._imu_heading_bearing = None
        self._update_cross_check()

    # ---------- 렌더: 위치 ----------
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
            self._fix_status.setText(f'{fix}  --  solution not usable')
            self._fix_status.setStyleSheet(f'color: {COLOR_BAD};')
        elif rtk == 'RTK FIXED':
            self._fix_status.setText(f'{fix}  --  {rtk}')
            self._fix_status.setStyleSheet(f'color: {COLOR_GOOD};')
        elif rtk == 'RTK FLOAT':
            self._fix_status.setText(f'{fix}  --  {rtk}')
            self._fix_status.setStyleSheet(f'color: {COLOR_OK};')
        else:
            self._fix_status.setText(fix)
            self._fix_status.setStyleSheet(f'color: {COLOR_OK};')

        # p_dop는 0.01 단위.
        self._fix_detail.setText(
            f'sats {msg.num_sv:2d}   pDOP {msg.p_dop / 100.0:.2f}   '
            f'speed {msg.g_speed / 1000.0:.2f} m/s')

    def _render_gps_odom(self, msg):
        self._timers['gps'].start()
        p = msg.pose.pose.position
        self._gps_pos = (p.x, p.y)
        self._gps_xy.setText(f'x {p.x:10.3f}   y {p.y:10.3f}')
        self._gps_xy.setStyleSheet(f'color: {COLOR_RAW};')
        self._update_separation()

    def _render_ekf_odom(self, msg):
        self._timers['ekf'].start()
        p = msg.pose.pose.position
        self._ekf_pos = (p.x, p.y)
        self._ekf_xy.setText(f'x {p.x:10.3f}   y {p.y:10.3f}')
        self._ekf_xy.setStyleSheet(f'color: {COLOR_EKF};')

        self._ekf_bearing = bearing_from_enu_yaw(
            yaw_from_quaternion(msg.pose.pose.orientation))
        self._compass.set_ekf(self._ekf_bearing)
        self._history.add_ekf(self._ekf_bearing)

        self._update_separation()
        self._update_ekf_track()

    def _update_separation(self):
        if self._gps_pos is None or self._ekf_pos is None:
            self._delta.setText('--')
            self._delta.setStyleSheet(f'color: {COLOR_STALE};')
            return
        dist = math.hypot(self._ekf_pos[0] - self._gps_pos[0],
                          self._ekf_pos[1] - self._gps_pos[1])
        self._delta.setText(f'{dist:.3f} m')
        self._delta.setStyleSheet(f'color: {accuracy_color(dist)};')

    # ---------- 렌더: 방위 ----------
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
        self._compass.set_raw(bearing, head_valid)
        self._history.add_raw(bearing, carrier)

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
            self._head_status.setText('NO GNSS FIX -- heading meaningless')
            self._head_status.setStyleSheet(f'color: {COLOR_BAD};')
        elif not rel_pos_valid or not head_valid:
            self._head_status.setText(
                f'HEADING NOT VALID YET  ({carrier_name} solution)')
            self._head_status.setStyleSheet(f'color: {COLOR_BAD};')
        elif carrier == NavRELPOSNED9.FLAGS_CARR_SOLN_FIXED:
            self._head_status.setText('RTK FIXED -- heading valid')
            self._head_status.setStyleSheet(f'color: {COLOR_GOOD};')
        elif carrier == NavRELPOSNED9.FLAGS_CARR_SOLN_FLOAT:
            self._head_status.setText('RTK FLOAT -- heading valid but coarse')
            self._head_status.setStyleSheet(f'color: {COLOR_OK};')
        else:
            self._head_status.setText(f'heading valid ({carrier_name} solution)')
            self._head_status.setStyleSheet(f'color: {COLOR_OK};')

        # 한 줄로 붙이면 열 너비를 넘겨 양끝이 잘린다 -- 두 줄로 나눠 찍는다.
        self._flags_detail.setText(
            f'fixOK={int(fix_ok)}  diffSoln={int(diff_soln)}  '
            f'relPosValid={int(rel_pos_valid)}\n'
            f'headValid={int(head_valid)}  isMoving={int(is_moving)}  '
            f'carrSoln={carrier_name}')

        self._update_cross_check()
        self._update_ekf_track()
        self._update_stability()

    def _render_imu_heading(self, msg):
        self._timers['imu_heading'].start()
        self._imu_heading_bearing = bearing_from_enu_yaw(
            yaw_from_quaternion(msg.orientation))
        self._update_cross_check()

    def _update_cross_check(self):
        if self._raw_bearing is None or self._imu_heading_bearing is None:
            self._cross_check.setText('--')
            self._cross_check.setStyleSheet(f'color: {COLOR_STALE};')
            return
        diff = angle_diff_deg(self._raw_bearing, self._imu_heading_bearing)
        self._cross_check.setText(
            f'raw {self._raw_bearing:6.1f}°   imu/heading '
            f'{self._imu_heading_bearing:6.1f}°   Δ{diff:+5.1f}°')
        # 0.1도 넘게 벌어지면 반올림 오차 수준을 넘어선 것 -- remap/변환을 의심.
        self._cross_check.setStyleSheet(
            f'color: {COLOR_GOOD if abs(diff) <= 0.1 else COLOR_BAD};')

    def _update_ekf_track(self):
        if self._raw_bearing is None or self._ekf_bearing is None:
            self._ekf_track.setText('--')
            self._ekf_track.setStyleSheet(f'color: {COLOR_STALE};')
            return
        diff = angle_diff_deg(self._ekf_bearing, self._raw_bearing)
        self._ekf_track.setText(f'EKF {self._ekf_bearing:6.1f}°   Δ{diff:+5.1f}°')
        # 여기 Δ는 0이어야 하는 값이 아니다 -- EKF는 자이로까지 섞으므로 몇 도는
        # 정상이다. 다만 계속 벌어지거나 수십 도가 되면 헤딩이 안 먹히는 것.
        # head_valid가 꺼져 있으면 애초에 비교 대상이 아니라 회색으로 둔다.
        if not self._raw_valid:
            self._ekf_track.setStyleSheet(f'color: {COLOR_STALE};')
        else:
            self._ekf_track.setStyleSheet(
                f'color: {COLOR_GOOD if abs(diff) <= 5.0 else (COLOR_OK if abs(diff) <= 20.0 else COLOR_BAD)};')

    def _update_stability(self):
        stddev = self._history.recent_stddev_deg()
        if stddev is None:
            self._stability.setText('--')
            self._stability.setStyleSheet(f'color: {COLOR_STALE};')
            return
        self._stability.setText(f'{stddev:.2f}°')
        self._stability.setStyleSheet(
            f'color: {COLOR_GOOD if stddev <= 1.0 else (COLOR_OK if stddev <= 5.0 else COLOR_BAD)};')

    # ---------- 렌더: IMU 링크 ----------
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


class RtkMonitorNode(Node):
    def __init__(self, window):
        super().__init__('gps_accuracy_gui')

        def param(name, default):
            return self.declare_parameter(
                name, default).get_parameter_value().string_value

        self.navpvt_topic = param('navpvt_topic', '/ublox_gps_node_base/navpvt')
        self.relposned_topic = param(
            'navrelposned_topic', '/ublox_gps_node_rover/navrelposned')
        self.imu_heading_topic = param('imu_heading_topic', '/imu/heading')
        self.gps_topic = param('gps_odom_topic', '/odometry/gps')
        self.ekf_topic = param('ekf_odom_topic', '/odometry/filtered_map')
        # hyper_ebimu(EBIMU-9DOFV5)가 내는 토픽. 링크 생존 확인에만 쓴다.
        self.imu_topic = param('imu_topic', '/imu')

        # ublox_gps는 navpvt/navrelposned/navheading을 모두 기본 QoS
        # (RELIABLE/KEEP_LAST)로 낸다 -- 여기서도 기본값으로 맞춘다.
        self.create_subscription(
            NavPVT, self.navpvt_topic, lambda m: window.navpvt.emit(m), 10)
        self.create_subscription(
            NavRELPOSNED9, self.relposned_topic,
            lambda m: window.relposned.emit(m), 10)
        self.create_subscription(
            Imu, self.imu_heading_topic, lambda m: window.imu_heading.emit(m), 10)
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
            f'RTK monitor: {self.navpvt_topic} | {self.relposned_topic} | '
            f'{self.imu_heading_topic} | {self.gps_topic} | {self.ekf_topic} | '
            f'{self.imu_topic}')


def main():
    rclpy.init(args=sys.argv)
    app = QApplication(sys.argv)
    # Qt가 SIGINT를 삼키므로 Ctrl-C가 먹도록 기본 핸들러로 되돌린다.
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    window = RtkMonitorWindow()
    node = RtkMonitorNode(window)
    window.set_topics(node.navpvt_topic, node.relposned_topic,
                      node.imu_heading_topic, node.gps_topic, node.ekf_topic,
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
