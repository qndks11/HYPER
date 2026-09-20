#!/usr/bin/env python3
# =====================================================================
# 편집 모드의 조작판: 점 편집 상태와 미션 라벨.
#
# 라벨 목록이 단순한 목록이 아닌 이유가 여기 있습니다. mission_loader는 로드 시점에
# 라벨을 최근접 웨이포인트로 스냅하고, 거리가 label_snap_tolerance_m를 넘으면
# 미션 전체를 거부합니다. 점을 옮기다 보면 그 선을 조용히 넘길 수 있으므로,
# 스냅 거리를 실시간으로 보여 주고 넘긴 라벨은 빨갛게 칠합니다.
#
# 진입 금지 구역(keepout:)도 여기서 고르고, 새로 그리고, 지우고, 이름을 바꿉니다.
# 꼭짓점 편집 자체는 캔버스에서 합니다(app_window).
# =====================================================================

from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtWidgets import (
    QAbstractItemView, QCheckBox, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QListWidget,
    QListWidgetItem, QPushButton, QVBoxLayout, QWidget)

from .. import formats, theme
from ..mission_model import KEEPOUT_MIN_POINTS


class EditPanel(QWidget):

    label_selected = Signal(str)
    label_cleared = Signal(str)
    label_deselected = Signal()
    cone_selected = Signal(str)
    cone_deselected = Signal()
    keepout_selected = Signal(int)
    keepout_new = Signal()
    keepout_delete = Signal()
    keepout_renamed = Signal(str)
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
        # currentItemChanged가 아니라 선택 변화에 겁니다. 목록이 마우스가 아닌 이유로
        # 포커스를 받으면(Tab, 대화상자 뒤 창 복귀) Qt가 첫 줄을 "현재 줄"로 잡는데,
        # 그걸 선택으로 받으면 아무도 안 고른 라벨이 골라져 다음 클릭에 옮겨집니다.
        self._list.itemSelectionChanged.connect(self._on_label_selection)
        self._list.setToolTip(
            '라벨을 고른 뒤 코스를 좌클릭하면 그 자리로 옮깁니다.\n'
            '고른 채로는 캔버스 클릭이 전부 배치입니다 -- 점을 끌기 전에 Esc 또는 선택 해제로 푸세요.\n'
            '색은 스냅 거리입니다: 초록 안전, 주황 주의, 빨강이면 미션 로드가 거부됩니다.')
        labels_layout.addWidget(self._list, stretch=1)

        self._place_hint = QLabel('')
        self._place_hint.setWordWrap(True)
        self._place_hint.setStyleSheet(f'color: {theme.COLOR_STALE};')
        labels_layout.addWidget(self._place_hint)

        label_buttons = QHBoxLayout()
        deselect = QPushButton('선택 해제')
        deselect.setToolTip('라벨 선택을 풉니다 (Esc).')
        deselect.clicked.connect(self._list.clearSelection)
        label_buttons.addWidget(deselect)
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

        # ---------------------------------------------------------- 콘
        # 콘은 라벨과 다릅니다: 스냅도, 코스 소속도, 허용 오차도 없습니다
        # (steps[].cases[].cone -- courses.<n>.labels가 아닙니다). 그래서 이 목록에는
        # 색으로 나타낼 상태가 없고, 저장은 "미션 저장" 버튼을 그대로 씁니다(라벨과
        # 한 번에 저장됩니다).
        cones = QGroupBox('미션 콘')
        cones_layout = QVBoxLayout(cones)
        self._cones_hint = QLabel('미션 파일이 열려 있지 않습니다.')
        self._cones_hint.setWordWrap(True)
        self._cones_hint.setStyleSheet(f'color: {theme.COLOR_STALE};')
        cones_layout.addWidget(self._cones_hint)

        self._cone_list = QListWidget()
        self._cone_list.setSelectionMode(QAbstractItemView.SingleSelection)
        self._cone_list.itemSelectionChanged.connect(self._on_cone_selection)
        self._cone_list.setToolTip(
            '분기 판정에 쓰는 콘 좌표입니다(select_by: clearance). 스냅도 허용 오차도\n'
            '없습니다 -- 고른 뒤 캔버스를 클릭하거나 끌면 그 좌표 그대로 옮겨집니다.\n'
            'Esc 또는 선택 해제로 풉니다.')
        cones_layout.addWidget(self._cone_list, stretch=1)

        cone_deselect = QPushButton('선택 해제')
        cone_deselect.setToolTip('콘 선택을 풉니다 (Esc).')
        cone_deselect.clicked.connect(self._cone_list.clearSelection)
        cones_layout.addWidget(cone_deselect)

        self._cone_place_hint = QLabel('')
        self._cone_place_hint.setWordWrap(True)
        self._cone_place_hint.setStyleSheet(f'color: {theme.COLOR_STALE};')
        cones_layout.addWidget(self._cone_place_hint)
        root.addWidget(cones, stretch=1)

        # ---------------------------------------------------------- 진입 금지 구역
        # 미션 파일의 최상위 keepout:. mission_manager가 이 다각형을 마스크로 구워 nav2
        # local_costmap(keepout_layer, 팽창됨)에 줍니다. 저장은 "미션 저장"을 그대로 씁니다.
        keepout = QGroupBox('진입 금지 구역')
        keepout_layout = QVBoxLayout(keepout)
        self._keepout_hint = QLabel('미션 파일이 열려 있지 않습니다.')
        self._keepout_hint.setWordWrap(True)
        self._keepout_hint.setStyleSheet(f'color: {theme.COLOR_STALE};')
        keepout_layout.addWidget(self._keepout_hint)

        self._zone_list = QListWidget()
        self._zone_list.setSelectionMode(QAbstractItemView.SingleSelection)
        self._zone_list.currentItemChanged.connect(self._on_zone_selected)
        self._zone_list.setToolTip(
            '구역을 고르면 꼭짓점 핸들이 뜹니다.\n'
            '  드래그: 꼭짓점 이동   Shift+좌클릭: 가장 가까운 변에 꼭짓점 삽입\n'
            '  Del: 고른 꼭짓점 삭제(3개까지)   Esc: 선택 해제\n'
            'nav2에는 lethal(254)로 들어갑니다 -- inflation을 받지 않으니 여유는 직접 그리세요.')
        keepout_layout.addWidget(self._zone_list, stretch=1)

        name_row = QHBoxLayout()
        name_row.addWidget(QLabel('이름'))
        self._zone_name = QLineEdit()
        self._zone_name.setEnabled(False)
        self._zone_name.editingFinished.connect(
            lambda: self.keepout_renamed.emit(self._zone_name.text()))
        name_row.addWidget(self._zone_name, stretch=1)
        keepout_layout.addLayout(name_row)

        zone_buttons = QHBoxLayout()
        self._new_zone = QPushButton('새 구역 그리기')
        self._new_zone.setToolTip(
            '캔버스를 클릭해 꼭짓점을 찍고 Enter / 더블클릭 / 첫 점 클릭으로 닫습니다.\n'
            'Esc: 취소.')
        self._new_zone.setEnabled(False)
        self._new_zone.clicked.connect(self.keepout_new.emit)
        self._delete_zone = QPushButton('구역 지우기')
        self._delete_zone.setEnabled(False)
        self._delete_zone.clicked.connect(self.keepout_delete.emit)
        zone_buttons.addWidget(self._new_zone)
        zone_buttons.addWidget(self._delete_zone)
        keepout_layout.addLayout(zone_buttons)

        self._zone_place_hint = QLabel('')
        self._zone_place_hint.setWordWrap(True)
        self._zone_place_hint.setStyleSheet(f'color: {theme.COLOR_STALE};')
        keepout_layout.addWidget(self._zone_place_hint)
        root.addWidget(keepout, stretch=1)

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
        """report는 mission_model.snap_report()의 결과입니다.

        라벨은 코스마다 독립이므로 코스별로 묶어 보여 주고, 목록의 식별자는 이름이
        아니라 formats.label_key(코스, 이름)입니다 -- 같은 이름이 두 코스에 있어도
        서로 다른 라벨입니다.
        """
        self._list.blockSignals(True)
        self._list.clear()
        if mission_name is None:
            self._labels_hint.setText('미션 파일이 열려 있지 않습니다.')
            self._save_mission.setEnabled(False)
            self._list.blockSignals(False)
            return

        placeable = [e for e in report if e[4] != 'sentinel']
        over = sum(1 for entry in report if entry[4] == 'over')
        missing = sum(1 for entry in report if entry[4] == 'missing')
        summary = f'{mission_name} -- 라벨 {len(placeable)}개'
        if missing:
            summary += f', 미배치 {missing}개'
        if over:
            summary += f', 허용 오차 초과 {over}개'
        self._labels_hint.setText(summary)
        self._labels_hint.setStyleSheet(
            f'color: {theme.COLOR_BAD if over else theme.COLOR_STALE};')

        orphan_keys = {formats.label_key(c, n) for c, n in orphans}
        current_course = None
        for course, name, index, distance, state in report:
            if course != current_course:
                current_course = course
                header = QListWidgetItem(f'— {course} —')
                header.setForeground(QColor(theme.COLOR_STALE))
                header.setFlags(Qt.NoItemFlags)
                self._list.addItem(header)

            key = formats.label_key(course, name)
            if state == 'sentinel':
                # `last`는 좌표가 아니라 "그 코스의 마지막 점"입니다. 옮길 수 없으므로
                # 고를 수도 없게 둡니다 -- 저장할 때는 그대로 다시 쓰입니다.
                text = f'{name}   (last)'
                color = theme.COLOR_STALE
            elif state == 'missing':
                text = f'{name}   (미배치)'
                color = theme.COLOR_STALE
            elif state == 'nocourse':
                text = f'{name}   (코스 미지정)'
                color = theme.COLOR_STALE
            else:
                text = f'{name}   wp #{index}   {distance:.2f} m'
                color = {'ok': theme.COLOR_GOOD, 'near': theme.COLOR_OK,
                         'over': theme.COLOR_BAD}[state]
            if key in orphan_keys:
                text += '   (orphan: 어떤 step도 참조 안 함)'
                color = theme.COLOR_STALE
            item = QListWidgetItem(text)
            item.setForeground(QColor(color))
            item.setData(Qt.UserRole, key)
            if state == 'sentinel':
                item.setFlags(Qt.ItemIsEnabled)
                item.setToolTip(
                    "`last` 라벨입니다. 좌표가 아니라 그 코스의 마지막 웨이포인트를 "
                    "뜻하므로 옮길 수 없습니다.")
            if state == 'over':
                item.setToolTip(
                    '이 거리면 mission_manager가 미션 로드를 거부합니다 '
                    '(label_snap_tolerance_m 초과).')
            self._list.addItem(item)

        self._select_row(self._list, active)
        self._save_mission.setEnabled(True)
        self._list.blockSignals(False)

    def set_place_hint(self, text):
        self._place_hint.setText(text)

    def current_label(self):
        # currentItem이 아니라 선택입니다 -- 포커스만 받은 "현재 줄"을 지우면 안 됩니다.
        items = self._list.selectedItems()
        return items[0].data(Qt.UserRole) if items else None

    def select_label(self, key):
        """캔버스에서 고르거나 푼 것을 목록에 맞춥니다. None이면 선택과 안내를 비웁니다.
        시그널을 막아 label_selected/label_deselected로 되돌아오지 않게 합니다."""
        self._select_row(self._list, key)
        if key is None:
            self._place_hint.setText('')

    def set_cones(self, mission, mission_name=None, active=None):
        """mission은 MissionModel입니다(cone_names()/cones로 콘을 읽습니다)."""
        self._cone_list.blockSignals(True)
        self._cone_list.clear()
        if mission_name is None:
            self._cones_hint.setText('미션 파일이 열려 있지 않습니다.')
            self._cone_list.blockSignals(False)
            return

        keys = mission.cone_names()
        self._cones_hint.setText(
            f'{mission_name} -- 콘 {len(keys)}개' if keys else f'{mission_name} -- 콘 없음')

        for key in keys:
            cone = mission.cones[key]
            item = QListWidgetItem(f"{cone['value']}   ({cone['x']:.2f}, {cone['y']:.2f})")
            item.setForeground(QColor(theme.COLOR_CONE))
            item.setData(Qt.UserRole, key)
            self._cone_list.addItem(item)

        self._select_row(self._cone_list, active)
        self._cone_list.blockSignals(False)

    def set_cone_place_hint(self, text):
        self._cone_place_hint.setText(text)

    def current_cone(self):
        items = self._cone_list.selectedItems()
        return items[0].data(Qt.UserRole) if items else None

    def select_cone(self, key):
        self._select_row(self._cone_list, key)
        if key is None:
            self._cone_place_hint.setText('')

    @staticmethod
    def _select_row(widget, key):
        """key인 줄을 고르고, 없으면(None 포함) 선택도 현재 줄도 비웁니다. 시그널은 막습니다."""
        blocked = widget.blockSignals(True)
        widget.clearSelection()
        widget.setCurrentRow(-1)
        if key is not None:
            for row in range(widget.count()):
                if widget.item(row).data(Qt.UserRole) == key:
                    widget.setCurrentRow(row)
                    widget.item(row).setSelected(True)
                    break
        widget.blockSignals(blocked)

    def set_keepout(self, zones, mission_name=None, active=None):
        """zones는 MissionModel.keepout, active는 고른 구역의 인덱스(없으면 None)."""
        self._zone_list.blockSignals(True)
        self._zone_list.clear()
        self._zone_name.blockSignals(True)
        if mission_name is None:
            self._keepout_hint.setText('미션 파일이 열려 있지 않습니다.')
            self._zone_name.setText('')
            self._zone_name.setEnabled(False)
            self._new_zone.setEnabled(False)
            self._delete_zone.setEnabled(False)
            self._zone_name.blockSignals(False)
            self._zone_list.blockSignals(False)
            return

        self._keepout_hint.setText(
            f'{mission_name} -- 구역 {len(zones)}개' if zones else f'{mission_name} -- 구역 없음')
        for index, zone in enumerate(zones):
            count = len(zone['points'])
            text = f"{zone['name']}   ({count}점)"
            color = theme.COLOR_KEEPOUT
            if count < KEEPOUT_MIN_POINTS:
                text += '   ** 3점 미만: 미션 로드가 거부됩니다 **'
                color = theme.COLOR_BAD
            item = QListWidgetItem(text)
            item.setForeground(QColor(color))
            item.setData(Qt.UserRole, index)
            self._zone_list.addItem(item)

        valid = active is not None and 0 <= active < len(zones)
        if valid:
            self._zone_list.setCurrentRow(active)
        self._zone_name.setText(zones[active]['name'] if valid else '')
        self._zone_name.setEnabled(valid)
        self._new_zone.setEnabled(True)
        self._delete_zone.setEnabled(valid)
        self._zone_name.blockSignals(False)
        self._zone_list.blockSignals(False)

    def set_zone_place_hint(self, text):
        self._zone_place_hint.setText(text)

    # ------------------------------------------------------------------ 이벤트
    def _on_label_selection(self):
        key = self.current_label()
        if key is not None:
            self.label_selected.emit(key)
        else:
            self.label_deselected.emit()

    def _on_clear(self):
        name = self.current_label()
        if name:
            self.label_cleared.emit(name)

    def _on_cone_selection(self):
        key = self.current_cone()
        if key is not None:
            self.cone_selected.emit(key)
        else:
            self.cone_deselected.emit()

    def _on_zone_selected(self, current, _previous):
        if current is not None:
            self.keepout_selected.emit(int(current.data(Qt.UserRole)))
