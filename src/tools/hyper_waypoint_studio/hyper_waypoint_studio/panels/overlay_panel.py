#!/usr/bin/env python3
# =====================================================================
# 배경 이미지와 그 배치.
#
# 지오레퍼런싱은 label_waypoints.py가 하던 세 가지를 그대로 옮겼습니다.
#   gazebo  : course.png + ground.obj의 쿼드 경계 (맞출 것이 없습니다)
#   align   : 항공사진 + <이미지>.align.yaml (중심/가로 폭/회전을 손으로 맞춥니다)
#   extent  : 이미 지오레퍼런스된 정사영상 + map 프레임 경계
# =====================================================================

import os

from python_qt_binding.QtCore import Signal
from python_qt_binding.QtWidgets import (
    QDoubleSpinBox, QFormLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton,
    QSlider, QVBoxLayout, QWidget)
from python_qt_binding.QtCore import Qt

from .. import theme

NUDGE_M = 2.0
NUDGE_FINE_M = 0.2
SCALE_STEP = 1.01
SCALE_FINE = 1.001
ROT_STEP = 0.5
ROT_FINE = 0.05


class OverlayPanel(QWidget):

    load_gazebo = Signal()
    load_image = Signal()
    clear_overlay = Signal()
    save_alignment = Signal()
    nudged = Signal(float, float, float, float)   # dx, dy, scale, rot
    alpha_changed = Signal(float)
    fit_requested = Signal()

    def __init__(self):
        super().__init__()
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)

        buttons = QHBoxLayout()
        gazebo = QPushButton('시뮬 코스')
        gazebo.setToolTip('hyper_gazebo의 course.png. 배치는 ground.obj에서 읽습니다')
        gazebo.clicked.connect(self.load_gazebo.emit)
        image = QPushButton('이미지…')
        image.setToolTip('항공사진/정사영상을 고릅니다. 배치는 .align.yaml 사이드카')
        image.clicked.connect(self.load_image.emit)
        clear = QPushButton('없애기')
        clear.clicked.connect(self.clear_overlay.emit)
        for button in (gazebo, image, clear):
            buttons.addWidget(button)
        root.addLayout(buttons)

        self._source = QLabel('배경 없음')
        self._source.setWordWrap(True)
        self._source.setStyleSheet(f'color: {theme.COLOR_STALE};')
        root.addWidget(self._source)

        alpha_row = QHBoxLayout()
        alpha_row.addWidget(QLabel('불투명도'))
        self._alpha = QSlider(Qt.Horizontal)
        self._alpha.setRange(10, 100)
        self._alpha.setValue(100)
        self._alpha.valueChanged.connect(
            lambda value: self.alpha_changed.emit(value / 100.0))
        alpha_row.addWidget(self._alpha, stretch=1)
        root.addLayout(alpha_row)

        self._align_box = QGroupBox('정렬')
        self._align_box.setToolTip(
            '항공사진처럼 지오레퍼런스가 없는 이미지를 코스에 맞춥니다.\n'
            '맞춘 뒤 "정렬 저장"을 눌러야 <이미지>.align.yaml에 남습니다.')
        align = QVBoxLayout(self._align_box)

        grid = QHBoxLayout()
        for caption, dx, dy in (('←', -1, 0), ('→', 1, 0), ('↑', 0, 1), ('↓', 0, -1)):
            button = QPushButton(caption)
            button.setFixedWidth(34)
            button.setToolTip(f'{NUDGE_M} m 이동 (Shift: {NUDGE_FINE_M} m)')
            button.clicked.connect(
                lambda _=False, x=dx, y=dy: self._move(x, y))
            grid.addWidget(button)
        align.addLayout(grid)

        scale_row = QHBoxLayout()
        for caption, factor in (('축소 -', 1.0 / SCALE_STEP), ('확대 +', SCALE_STEP)):
            button = QPushButton(caption)
            button.setToolTip('축척 ±1% (Shift: ±0.1%)')
            button.clicked.connect(lambda _=False, f=factor: self._scale(f))
            scale_row.addWidget(button)
        align.addLayout(scale_row)

        rot_row = QHBoxLayout()
        for caption, delta in (('↺ ,', ROT_STEP), ('↻ .', -ROT_STEP)):
            button = QPushButton(caption)
            button.setToolTip('회전 ±0.5° (Shift: ±0.05°)')
            button.clicked.connect(lambda _=False, d=delta: self._rotate(d))
            rot_row.addWidget(button)
        align.addLayout(rot_row)

        self._readout = QLabel('-')
        self._readout.setStyleSheet(f'color: {theme.COLOR_STALE};')
        self._readout.setWordWrap(True)
        align.addWidget(self._readout)

        tail = QHBoxLayout()
        fit = QPushButton('화면 맞춤')
        fit.clicked.connect(self.fit_requested.emit)
        self._save = QPushButton('정렬 저장')
        self._save.clicked.connect(self.save_alignment.emit)
        tail.addWidget(fit)
        tail.addWidget(self._save)
        align.addLayout(tail)

        root.addWidget(self._align_box)
        root.addStretch(1)
        self.set_overlay(None, alignable=False)

    # ------------------------------------------------------------------ 상태
    def set_overlay(self, path, alignable, cx=0.0, cy=0.0, width_m=0.0, rot=0.0,
                    dirty=False):
        if path is None:
            self._source.setText('배경 없음')
            self._align_box.setEnabled(False)
            self._readout.setText('-')
            return
        self._source.setText(os.path.basename(path))
        self._source.setToolTip(path)
        self._align_box.setEnabled(alignable)
        if not alignable:
            self._readout.setText('ground.obj가 배치를 정합니다 -- 맞출 것이 없습니다.')
            return
        star = ' *' if dirty else ''
        self._readout.setText(
            f'중심 ({cx:.2f}, {cy:.2f})\n폭 {width_m:.2f} m   회전 {rot:.2f}°{star}')
        self._save.setEnabled(dirty)

    # ------------------------------------------------------------------ 조작
    @staticmethod
    def _fine():
        from python_qt_binding.QtWidgets import QApplication
        return bool(QApplication.keyboardModifiers() & Qt.ShiftModifier)

    def _move(self, sx, sy):
        step = NUDGE_FINE_M if self._fine() else NUDGE_M
        self.nudged.emit(sx * step, sy * step, 1.0, 0.0)

    def _scale(self, factor):
        if self._fine():
            factor = SCALE_FINE if factor > 1.0 else 1.0 / SCALE_FINE
        self.nudged.emit(0.0, 0.0, factor, 0.0)

    def _rotate(self, delta):
        if self._fine():
            delta = ROT_FINE if delta > 0 else -ROT_FINE
        self.nudged.emit(0.0, 0.0, 1.0, delta)
