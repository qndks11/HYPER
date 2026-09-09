#!/usr/bin/env python3
# =====================================================================
# 녹화 조작판. waypoint_record_gui.py가 하던 일을 그대로 하되, 미니맵 대신 공용
# 캔버스에 그립니다 -- 새로 따는 코스를 배경 이미지와 이전 코스 위에 겹쳐 보게
# 하려는 것이 이 통합의 목적입니다.
#
# 핵심 안전 성질은 그대로입니다: 기록 시작이 곧 CSV truncate이므로, Record를 누르기
# 전까지 이전 녹화본은 그대로 남아 있고, 무엇을 덮어쓰는지 눌러 보기 전에 보입니다.
# =====================================================================

from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import (
    QFrame, QGridLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton,
    QVBoxLayout, QWidget)

from .. import theme

FIELDS = (
    ('points', '기록된 점'),
    ('length', '누적 거리'),
    ('since_last', '다음 점까지'),
    ('pose', '현재 위치 (map)'),
    ('speed', '속도'),
    ('gps', 'GPS 상태'),
    ('cov', 'EKF 공분산 xx / yy'),
)


class RecordPanel(QWidget):

    start_requested = Signal()
    stop_requested = Signal()
    browse_requested = Signal()
    destination_changed = Signal(str)

    def __init__(self):
        super().__init__()
        self._recording = False

        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)

        self._state = QLabel('연결 대기')
        self._state.setAlignment(Qt.AlignCenter)
        self._state.setFont(QFont('DejaVu Sans', 26, QFont.Bold))
        self._state.setStyleSheet(f'color: {theme.COLOR_STALE};')
        root.addWidget(self._state)

        self._file = QLabel('-')
        self._file.setAlignment(Qt.AlignCenter)
        self._file.setStyleSheet(f'color: {theme.COLOR_STALE};')
        self._file.setWordWrap(True)
        root.addWidget(self._file)

        row = QHBoxLayout()
        row.addWidget(QLabel('저장 파일'))
        self._destination = QLineEdit()
        self._destination.setPlaceholderText('waypoint_record.csv')
        self._destination.setFont(QFont('DejaVu Sans Mono', 10))
        self._destination.editingFinished.connect(
            lambda: self.destination_changed.emit(self._destination.text().strip()))
        row.addWidget(self._destination, stretch=1)
        browse = QPushButton('…')
        browse.setFixedWidth(30)
        browse.clicked.connect(self.browse_requested.emit)
        row.addWidget(browse)
        root.addLayout(row)

        buttons = QHBoxLayout()
        self._record = QPushButton('● Record')
        self._stop = QPushButton('■ Stop')
        for button in (self._record, self._stop):
            button.setFont(QFont('DejaVu Sans', 13, QFont.Bold))
            button.setMinimumHeight(46)
            buttons.addWidget(button)
        self._record.clicked.connect(self.start_requested.emit)
        self._stop.clicked.connect(self.stop_requested.emit)
        root.addLayout(buttons)

        frame = QFrame()
        frame.setStyleSheet(
            f'background-color: {theme.COLOR_PANEL}; border-radius: 6px;')
        grid = QGridLayout(frame)
        self._fields = {}
        for row_index, (key, caption) in enumerate(FIELDS):
            caption_label = QLabel(caption)
            caption_label.setStyleSheet(f'color: {theme.COLOR_STALE};')
            value = QLabel('-')
            value.setFont(QFont('DejaVu Sans Mono', 11, QFont.Bold))
            grid.addWidget(caption_label, row_index, 0)
            grid.addWidget(value, row_index, 1, alignment=Qt.AlignRight)
            self._fields[key] = value
        root.addWidget(frame)

        self._message = QLabel(' ')
        self._message.setWordWrap(True)
        self._message.setStyleSheet(f'color: {theme.COLOR_STALE};')
        root.addWidget(self._message)
        root.addStretch(1)

    # ------------------------------------------------------------------ 상태
    @property
    def recording(self):
        return self._recording

    def destination(self):
        return self._destination.text().strip()

    def set_destination(self, path):
        self._destination.setText(path or '')

    def show_message(self, text, color=None):
        self._message.setText(text)
        self._message.setStyleSheet(f'color: {color or theme.COLOR_STALE};')

    def set_disconnected(self, reason):
        self._state.setText('ROS 없음')
        self._state.setStyleSheet(f'color: {theme.COLOR_STALE};')
        self._record.setEnabled(False)
        self._stop.setEnabled(False)
        self.show_message(reason, theme.COLOR_STALE)

    def mark_stale(self):
        self._state.setText('상태 없음')
        self._state.setStyleSheet(f'color: {theme.COLOR_STALE};')

    def update_status(self, status, as_float):
        """레코더의 key=value 한 줄을 화면에 풉니다."""
        self._recording = status.get('recording') == '1'
        if self._recording:
            self._state.setText('● REC')
            self._state.setStyleSheet(f'color: {theme.COLOR_BAD};')
        else:
            self._state.setText('IDLE')
            self._state.setStyleSheet(f'color: {theme.COLOR_STALE};')
        self._record.setEnabled(not self._recording)
        self._stop.setEnabled(self._recording)
        self._file.setText(status.get('file', '-'))

        points = status.get('points', '0')
        spacing = as_float(status, 'spacing')
        self._fields['points'].setText(
            f'{points} 점' + (f'  (간격 {spacing:.2f} m)' if spacing is not None else ''))

        length = as_float(status, 'length')
        self._fields['length'].setText('-' if length is None else f'{length:.1f} m')

        since = as_float(status, 'since_last')
        if since is None or not self._recording:
            self._fields['since_last'].setText('-')
            self._fields['since_last'].setStyleSheet(f'color: {theme.COLOR_STALE};')
        else:
            remaining = max((spacing or 0.0) - since, 0.0)
            self._fields['since_last'].setText(
                f'{since:.2f} m  (남은 {remaining:.2f} m)')
            self._fields['since_last'].setStyleSheet(
                f'color: {theme.COLOR_GOOD if remaining <= 0.0 else theme.COLOR_TEXT};')

        x = as_float(status, 'x')
        y = as_float(status, 'y')
        pose_age = as_float(status, 'pose_age', 0.0)
        if x is None or y is None:
            self._fields['pose'].setText('/odometry/filtered_map 없음')
            self._fields['pose'].setStyleSheet(f'color: {theme.COLOR_BAD};')
        else:
            fresh = pose_age is not None and pose_age <= theme.STALE_S
            self._fields['pose'].setText(f'{x:.2f}, {y:.2f}')
            self._fields['pose'].setStyleSheet(
                f'color: {theme.COLOR_TEXT if fresh else theme.COLOR_STALE};')

        speed = as_float(status, 'speed')
        self._fields['speed'].setText('-' if speed is None else f'{speed:+.2f} m/s')

        gps_status = status.get('gps_status')
        gps_age = as_float(status, 'gps_age')
        if gps_status is None:
            self._fields['gps'].setText('/gps/fix 없음')
            self._fields['gps'].setStyleSheet(f'color: {theme.COLOR_BAD};')
        else:
            try:
                caption, color = theme.GPS_STATUS[int(gps_status)]
            except (KeyError, ValueError):
                caption, color = f'? ({gps_status})', theme.COLOR_OK
            if gps_age is not None and gps_age > theme.STALE_S:
                caption, color = f'{caption} (stale {gps_age:.0f}s)', theme.COLOR_STALE
            self._fields['gps'].setText(caption)
            self._fields['gps'].setStyleSheet(f'color: {color};')

        cov_xx = as_float(status, 'cov_xx')
        cov_yy = as_float(status, 'cov_yy')
        if cov_xx is None or cov_yy is None:
            self._fields['cov'].setText('-')
        else:
            self._fields['cov'].setText(f'{cov_xx:.3f} / {cov_yy:.3f}')
            self._fields['cov'].setStyleSheet(
                f'color: {theme.COLOR_GOOD if max(cov_xx, cov_yy) < 0.1 else theme.COLOR_OK};')
