#!/usr/bin/env python3
# =====================================================================
# 주행 모드: 미션을 중간 스텝부터 돌립니다.
#
# 왜 필요한가: 평행 주차나 완주 같은 미션 후반 스텝 하나를 고치고 확인하려고 코스를
# 처음부터 도는 것은 낭비입니다. mission_manager의 '~/start'는 "현재 스텝부터"라서,
# 현재 스텝을 원하는 곳으로 옮기는 '~/goto_step'만 있으면 됩니다.
#
# 스텝 목록의 출처가 둘인 이유: mission_loader는 steps를 펼친 뒤 routes의 스텝을
# 그 뒤에 덧붙이므로, yaml만 봐서는 갈래(route) 스텝의 인덱스를 알 수 없습니다.
# 그래서 노드가 내보내는 ~/steps를 우선 쓰고, 없으면 yaml 순서로 떨어집니다.
# =====================================================================

from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtGui import QColor, QFont
from python_qt_binding.QtWidgets import (
    QAbstractItemView, QCheckBox, QDoubleSpinBox, QGroupBox, QHBoxLayout, QLabel,
    QListWidget, QListWidgetItem, QPushButton, QVBoxLayout, QWidget)

from .. import theme


class DrivePanel(QWidget):

    goto_and_start = Signal(int, str)   # step_index, step_label
    goto_only = Signal(int, str)
    teleport_requested = Signal(str, float)   # label, offset_m
    start = Signal()
    cancel = Signal()
    skip = Signal()
    restart = Signal()

    def __init__(self):
        super().__init__()
        self._steps = []
        self._from_node = False

        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)

        self._status = QLabel('mission_manager 없음')
        self._status.setWordWrap(True)
        self._status.setAlignment(Qt.AlignCenter)
        self._status.setFont(QFont('DejaVu Sans', 11, QFont.Bold))
        self._status.setStyleSheet(
            f'background-color: #3a3a3a; color: white; padding: 6px; border-radius: 4px;')
        root.addWidget(self._status)

        controls = QHBoxLayout()
        for caption, signal, tip in (
                ('시작', self.start, '현재 스텝부터 시작/재개합니다'),
                ('취소', self.cancel, '진행 중인 목표를 취소하고 그 스텝에서 대기'),
                ('건너뛰기', self.skip, '현재 스텝을 포기하고 다음으로'),
                ('처음으로', self.restart, '스텝 0으로 되돌립니다')):
            button = QPushButton(caption)
            button.setToolTip(tip)
            button.clicked.connect(signal.emit)
            controls.addWidget(button)
            setattr(self, f'_button_{caption}', button)
        root.addLayout(controls)

        box = QGroupBox('스텝')
        box_layout = QVBoxLayout(box)
        self._source = QLabel('미션 파일이 열려 있지 않습니다.')
        self._source.setWordWrap(True)
        self._source.setStyleSheet(f'color: {theme.COLOR_STALE};')
        box_layout.addWidget(self._source)

        self._list = QListWidget()
        self._list.setSelectionMode(QAbstractItemView.SingleSelection)
        self._list.setToolTip('스텝을 고른 뒤 "여기서 시작"을 누르면 그 스텝부터 돕니다.')
        box_layout.addWidget(self._list, stretch=1)

        teleport_row = QHBoxLayout()
        self._teleport = QCheckBox('먼저 순간이동')
        self._teleport.setToolTip(
            '시뮬레이션 전용입니다. 스텝의 라벨 위치로 차를 옮긴 뒤 시작합니다.\n'
            '실차에서는 차를 직접 그 지점에 가져다 놓으세요.')
        teleport_row.addWidget(self._teleport)
        teleport_row.addWidget(QLabel('오프셋'))
        self._offset = QDoubleSpinBox()
        self._offset.setRange(-100.0, 100.0)
        self._offset.setSingleStep(1.0)
        self._offset.setValue(-10.0)
        self._offset.setSuffix(' m')
        self._offset.setToolTip('음수 = 라벨 앞쪽(아직 도달 전). 스텝을 처음부터 보려면 -10쯤.')
        teleport_row.addWidget(self._offset)
        box_layout.addLayout(teleport_row)

        buttons = QHBoxLayout()
        self._goto = QPushButton('여기로 이동')
        self._goto.setToolTip('현재 스텝만 바꾸고 출발은 하지 않습니다')
        self._goto.clicked.connect(lambda: self._emit(self.goto_only))
        self._begin = QPushButton('여기서 시작')
        self._begin.setToolTip('그 스텝으로 옮긴 뒤 바로 출발합니다')
        self._begin.clicked.connect(lambda: self._emit(self.goto_and_start))
        buttons.addWidget(self._goto)
        buttons.addWidget(self._begin)
        box_layout.addLayout(buttons)

        self._caveat = QLabel(
            'drive 스텝의 시작 인덱스는 로드 시점에 정해집니다 -- n번 스텝으로 뛰면 '
            '경로는 n-1번이 끝난 자리에서 시작합니다. 차가 거기 없으면 먼저 그리로 갑니다.')
        self._caveat.setWordWrap(True)
        self._caveat.setStyleSheet(f'color: {theme.COLOR_STALE};')
        box_layout.addWidget(self._caveat)
        root.addWidget(box, stretch=1)

        self._message = QLabel(' ')
        self._message.setWordWrap(True)
        self._message.setStyleSheet(f'color: {theme.COLOR_STALE};')
        root.addWidget(self._message)

    # ------------------------------------------------------------------ 스텝
    def set_steps_from_mission(self, mission):
        """yaml에서 읽은 스텝. route 스텝은 인덱스를 모르므로 라벨로만 갑니다."""
        if self._from_node:
            return
        if mission is None:
            self._steps = []
            self._list.clear()
            self._source.setText('미션 파일이 열려 있지 않습니다.')
            return
        self._steps = [(i, kind, until, course, route)
                       for (i, kind, until, course, route) in mission.steps()]
        self._source.setText(
            f'{mission.name} -- {len(self._steps)} 스텝 (yaml에서 읽음; '
            'branch의 갈래 스텝은 노드가 떠야 보입니다)')
        self._fill([mission.step_summary(s) for s in self._steps])

    def set_steps_from_node(self, steps):
        """mission_manager가 펼친 목록. 갈래까지 포함한 진짜 인덱스입니다."""
        self._from_node = True
        self._steps = steps
        self._source.setText(f'mission_manager -- {len(steps)} 스텝')
        summaries = []
        for index, kind, label, course, route in steps:
            text = f'[{index + 1}] {kind}'
            if label:
                text += f' until={label}'
            if course and course not in ('', 'main'):
                text += f' course={course}'
            if route:
                text += f'  <{route}>'
            summaries.append(text)
        self._fill(summaries)

    def _fill(self, summaries):
        current = self._list.currentRow()
        self._list.clear()
        for text in summaries:
            self._list.addItem(QListWidgetItem(text))
        if 0 <= current < self._list.count():
            self._list.setCurrentRow(current)

    def _emit(self, signal):
        row = self._list.currentRow()
        if not 0 <= row < len(self._steps):
            self.show_message('먼저 스텝을 고르세요.', theme.COLOR_OK)
            return
        index, _kind, label, _course, _route = self._steps[row]
        if self._teleport.isChecked() and label:
            self.teleport_requested.emit(label, float(self._offset.value()))
        signal.emit(index, label or '')

    def current_step_label(self):
        row = self._list.currentRow()
        if not 0 <= row < len(self._steps):
            return None
        return self._steps[row][2]

    # ------------------------------------------------------------------ 상태
    def highlight(self, index):
        """지금 실행 중인 스텝을 굵게. 목록에서 눈으로 따라가기 위한 것입니다."""
        for row in range(self._list.count()):
            item = self._list.item(row)
            font = item.font()
            active = row < len(self._steps) and self._steps[row][0] == index
            font.setBold(active)
            item.setFont(font)
            item.setForeground(
                QColor(theme.COLOR_GOOD if active else theme.COLOR_TEXT))

    def set_status(self, text):
        """mission_manager의 status를 hyper_rqt 패널과 같은 색 규칙으로 보여 줍니다."""
        lowered = text.lower()
        if 'fail' in lowered:
            color = '#a32424'
        elif 'block' in lowered or 'wait' in lowered:
            color = '#a86a00'
        elif lowered.startswith('idle') or 'cancel' in lowered:
            color = '#3a3a3a'
        elif 'finish' in lowered:
            color = '#1f5c8b'
        else:
            color = '#1b7f3b'
        self._status.setText(text)
        self._status.setStyleSheet(
            f'background-color: {color}; color: white; padding: 6px; border-radius: 4px;')

        # "[3/21] drive ..." 에서 현재 스텝을 뽑아 목록에 표시합니다.
        if text.startswith('['):
            try:
                head = text[1:text.index(']')]
                self.highlight(int(head.split('/')[0]) - 1)
            except (ValueError, IndexError):
                pass

    def set_services_ready(self, ready, reason=''):
        for caption in ('시작', '취소', '건너뛰기', '처음으로'):
            button = getattr(self, f'_button_{caption}', None)
            if button is not None:
                button.setEnabled(ready)
                button.setToolTip(button.toolTip() if ready else reason)
        self._goto.setEnabled(ready)
        self._begin.setEnabled(ready)

    def show_message(self, text, color=None):
        self._message.setText(text)
        self._message.setStyleSheet(f'color: {color or theme.COLOR_STALE};')
