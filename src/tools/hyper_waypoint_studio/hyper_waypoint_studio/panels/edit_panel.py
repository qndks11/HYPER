#!/usr/bin/env python3
# =====================================================================
# 편집 모드의 조작판: 점 편집 상태와 미션 라벨.
#
# 라벨 목록이 단순한 목록이 아닌 이유가 여기 있습니다. mission_loader는 로드 시점에
# 라벨을 최근접 웨이포인트로 스냅하고, 거리가 label_snap_tolerance_m를 넘으면
# 미션 전체를 거부합니다. 점을 옮기다 보면 그 선을 조용히 넘길 수 있으므로,
# 스냅 거리를 실시간으로 보여 주고 넘긴 라벨은 빨갛게 칠합니다.
# =====================================================================

from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtWidgets import (
    QAbstractItemView, QCheckBox, QGroupBox, QHBoxLayout, QLabel, QListWidget,
    QListWidgetItem, QPushButton, QVBoxLayout, QWidget)

from .. import theme


class EditPanel(QWidget):

    label_selected = Signal(str)
    label_cleared = Signal(str)
    save_mission = Signal()
    save_course = Signal()
    save_course_as = Signal()
    undo = Signal()
    redo = Signal()
    flip_toggled = Signal()
    recompute_headings = Signal()
    insert_mode_changed = Signal(bool)

    def __init__(self):
        super().__init__()
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)

        # ---------------------------------------------------------- 점 편집
        points = QGroupBox('점 편집')
        points_layout = QVBoxLayout(points)
        self._hint = QLabel(
            '좌클릭 드래그로 점 이동.\n'
            'Del: 선택한 점 삭제.\n'
            'Shift+좌클릭: 가장 가까운 구간에 점 삽입.')
        self._hint.setStyleSheet(f'color: {theme.COLOR_STALE};')
        points_layout.addWidget(self._hint)

        self._selected = QLabel('선택된 점 없음')
        points_layout.addWidget(self._selected)

        undo_row = QHBoxLayout()
        self._undo = QPushButton('되돌리기')
        self._undo.clicked.connect(self.undo.emit)
        self._redo = QPushButton('다시')
        self._redo.clicked.connect(self.redo.emit)
        undo_row.addWidget(self._undo)
        undo_row.addWidget(self._redo)
        points_layout.addLayout(undo_row)

        self._flip = QPushButton('전진/후진 뒤집기')
        self._flip.setToolTip(
            '이 점이 후진으로 녹화됐는지의 표시를 손으로 바꿉니다.\n'
            '주차 구간처럼 차체 헤딩이 진행 방향의 반대인 곳에서\n'
            '자동 판정이 틀렸을 때만 쓰세요.')
        self._flip.clicked.connect(self.flip_toggled.emit)
        points_layout.addWidget(self._flip)

        self._recompute = QPushButton('헤딩 전부 다시 계산…')
        self._recompute.setToolTip(
            '녹화된 yaw를 버리고 경로 진행 방향에서 다시 구합니다.\n'
            '듀얼 RTK 기선이 FIXED가 아닌 채로 녹화해 파일 전체의 방위가\n'
            '일정하게 틀어졌을 때 씁니다. 전진/후진 표시와 방향 전환점은 보존됩니다.')
        self._recompute.clicked.connect(self.recompute_headings.emit)
        points_layout.addWidget(self._recompute)

        self._offset_note = QLabel('')
        self._offset_note.setWordWrap(True)
        self._offset_note.setStyleSheet(f'color: {theme.COLOR_STALE};')
        points_layout.addWidget(self._offset_note)

        self._reverse_note = QLabel('')
        self._reverse_note.setWordWrap(True)
        self._reverse_note.setStyleSheet(f'color: {theme.COLOR_STALE};')
        points_layout.addWidget(self._reverse_note)

        save_row = QHBoxLayout()
        save = QPushButton('코스 저장')
        save.clicked.connect(self.save_course.emit)
        save_as = QPushButton('다른 이름으로…')
        save_as.setToolTip('기본 위치는 waypoints/track/ -- 원본 녹화본은 그대로 둡니다')
        save_as.clicked.connect(self.save_course_as.emit)
        save_row.addWidget(save)
        save_row.addWidget(save_as)
        points_layout.addLayout(save_row)
        root.addWidget(points)

        # ---------------------------------------------------------- 라벨
        labels = QGroupBox('미션 라벨')
        labels_layout = QVBoxLayout(labels)
        self._labels_hint = QLabel('미션 파일이 열려 있지 않습니다.')
        self._labels_hint.setWordWrap(True)
        self._labels_hint.setStyleSheet(f'color: {theme.COLOR_STALE};')
        labels_layout.addWidget(self._labels_hint)

        self._list = QListWidget()
        self._list.setSelectionMode(QAbstractItemView.SingleSelection)
        self._list.currentItemChanged.connect(self._on_label_selected)
        self._list.setToolTip(
            '라벨을 고른 뒤 코스를 좌클릭하면 그 자리로 옮깁니다.\n'
            '색은 스냅 거리입니다: 초록 안전, 주황 주의, 빨강이면 미션 로드가 거부됩니다.')
        labels_layout.addWidget(self._list, stretch=1)

        self._place_hint = QLabel('')
        self._place_hint.setWordWrap(True)
        self._place_hint.setStyleSheet(f'color: {theme.COLOR_STALE};')
        labels_layout.addWidget(self._place_hint)

        label_buttons = QHBoxLayout()
        clear = QPushButton('라벨 지우기')
        clear.clicked.connect(self._on_clear)
        self._save_mission = QPushButton('미션 저장')
        self._save_mission.setToolTip(
            'labels 블록만 바이트 단위로 갈아끼웁니다 -- 주석과 steps는 그대로입니다.')
        self._save_mission.clicked.connect(self.save_mission.emit)
        label_buttons.addWidget(clear)
        label_buttons.addWidget(self._save_mission)
        labels_layout.addLayout(label_buttons)
        root.addWidget(labels, stretch=1)

    # ------------------------------------------------------------------ 갱신
    def set_selected_point(self, index, x=None, y=None, yaw=None, reverse=False):
        if index is None:
            self._selected.setText('선택된 점 없음')
            self._flip.setEnabled(False)
            return
        import math
        self._selected.setText(
            f'#{index}   ({x:.3f}, {y:.3f})   yaw {math.degrees(yaw):+.1f}°'
            + ('   [후진 녹화]' if reverse else ''))
        self._selected.setStyleSheet(
            f'color: {theme.COLOR_BAD if reverse else theme.COLOR_TEXT};')
        self._flip.setEnabled(True)

    def set_heading_offset(self, degrees, points):
        """파일 전체의 헤딩 틀어짐을 알려 줍니다. 크면 녹화 자체를 의심해야 합니다."""
        if points == 0:
            self._offset_note.setText('')
            return
        bad = abs(degrees) > 10.0
        self._offset_note.setText(
            f'경로 대비 헤딩 중앙값 {degrees:+.1f}°'
            + ('  ** 파일 전체가 틀어져 있습니다 -- 녹화 때 RTK 기선이 FIXED가 '
               '아니었을 가능성이 큽니다 **' if bad else ''))
        self._offset_note.setStyleSheet(
            f'color: {theme.COLOR_BAD if bad else theme.COLOR_STALE};')

    def set_undo_state(self, can_undo, can_redo):
        self._undo.setEnabled(can_undo)
        self._redo.setEnabled(can_redo)

    def set_reverse_note(self, text, bad=False):
        self._reverse_note.setText(text)
        self._reverse_note.setStyleSheet(
            f'color: {theme.COLOR_BAD if bad else theme.COLOR_STALE};')

    def set_labels(self, report, mission_name=None, active=None, orphans=()):
        """report는 mission_model.snap_report()의 결과입니다."""
        self._list.blockSignals(True)
        self._list.clear()
        if mission_name is None:
            self._labels_hint.setText('미션 파일이 열려 있지 않습니다.')
            self._save_mission.setEnabled(False)
            self._list.blockSignals(False)
            return

        over = sum(1 for entry in report if entry[3] == 'over')
        missing = sum(1 for entry in report if entry[3] == 'missing')
        summary = f'{mission_name} -- 라벨 {len(report)}개'
        if missing:
            summary += f', 미배치 {missing}개'
        if over:
            summary += f', 허용 오차 초과 {over}개'
        self._labels_hint.setText(summary)
        self._labels_hint.setStyleSheet(
            f'color: {theme.COLOR_BAD if over else theme.COLOR_STALE};')

        for name, index, distance, state in report:
            if state == 'missing':
                text = f'{name}   (미배치)'
                color = theme.COLOR_STALE
            elif state == 'nocourse':
                text = f'{name}   (코스 미지정)'
                color = theme.COLOR_STALE
            else:
                text = f'{name}   wp #{index}   {distance:.2f} m'
                color = {'ok': theme.COLOR_GOOD, 'near': theme.COLOR_OK,
                         'over': theme.COLOR_BAD}[state]
            item = QListWidgetItem(text)
            item.setForeground(QColor(color))
            item.setData(Qt.UserRole, name)
            if state == 'over':
                item.setToolTip(
                    '이 거리면 mission_manager가 미션 로드를 거부합니다 '
                    '(label_snap_tolerance_m 초과).')
            self._list.addItem(item)

        for name in orphans:
            item = QListWidgetItem(f'{name}   (orphan: 어떤 step도 참조 안 함)')
            item.setForeground(QColor(theme.COLOR_STALE))
            item.setData(Qt.UserRole, name)
            self._list.addItem(item)

        if active is not None:
            for row in range(self._list.count()):
                if self._list.item(row).data(Qt.UserRole) == active:
                    self._list.setCurrentRow(row)
                    break
        self._save_mission.setEnabled(True)
        self._list.blockSignals(False)

    def set_place_hint(self, text):
        self._place_hint.setText(text)

    def current_label(self):
        item = self._list.currentItem()
        return item.data(Qt.UserRole) if item else None

    # ------------------------------------------------------------------ 이벤트
    def _on_label_selected(self, current, _previous):
        if current is not None:
            self.label_selected.emit(current.data(Qt.UserRole))

    def _on_clear(self):
        name = self.current_label()
        if name:
            self.label_cleared.emit(name)
