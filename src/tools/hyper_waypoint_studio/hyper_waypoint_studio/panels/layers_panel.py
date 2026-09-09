#!/usr/bin/env python3
# =====================================================================
# 올라온 코스 목록. 탭이 아니라 한 화면에 겹쳐 그리므로, 여기서 하는 일은
# "무엇을 보이게 할지"와 "어느 코스가 미션의 어느 코스인지"입니다.
#
# 미션 코스 묶기가 중요한 이유: 라벨 스냅 거리는 자기 코스에 대고 재야만 의미가
# 있습니다. 엉뚱한 코스에 대고 잰 거리는 없는 것보다 나쁩니다.
# =====================================================================

from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtGui import QColor, QIcon, QPixmap
from python_qt_binding.QtWidgets import (
    QAbstractItemView, QComboBox, QHBoxLayout, QLabel, QPushButton,
    QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)

from .. import theme


def _swatch(color):
    pixmap = QPixmap(12, 12)
    pixmap.fill(QColor(color))
    return QIcon(pixmap)


class LayersPanel(QWidget):

    add_requested = Signal()
    remove_requested = Signal(int)
    active_changed = Signal(int)
    visibility_changed = Signal(int, bool)
    binding_changed = Signal(int, str)

    COL_VISIBLE, COL_NAME, COL_POINTS, COL_COURSE = range(4)

    def __init__(self):
        super().__init__()
        self._courses = []
        self._mission_courses = ['(none)']
        self._loading = False

        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)

        buttons = QHBoxLayout()
        add = QPushButton('코스 추가…')
        add.setToolTip('웨이포인트 CSV를 골라 화면에 겹쳐 올립니다')
        add.clicked.connect(self.add_requested.emit)
        self._remove = QPushButton('화면에서 빼기')
        self._remove.setToolTip('보기에서만 뺍니다 -- 파일은 건드리지 않습니다')
        self._remove.clicked.connect(self._on_remove)
        buttons.addWidget(add)
        buttons.addWidget(self._remove)
        root.addLayout(buttons)

        self._table = QTableWidget(0, 4)
        self._table.setHorizontalHeaderLabels(['', '파일', '점', '미션 코스'])
        self._table.verticalHeader().setVisible(False)
        self._table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._table.setSelectionMode(QAbstractItemView.SingleSelection)
        self._table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._table.setColumnWidth(self.COL_VISIBLE, 26)
        self._table.setColumnWidth(self.COL_NAME, 150)
        self._table.setColumnWidth(self.COL_POINTS, 52)
        self._table.itemChanged.connect(self._on_item_changed)
        self._table.itemSelectionChanged.connect(self._on_selection)
        root.addWidget(self._table, stretch=1)

        self._hint = QLabel('편집은 선택된 코스에만 적용됩니다.')
        self._hint.setWordWrap(True)
        self._hint.setStyleSheet(f'color: {theme.COLOR_STALE};')
        root.addWidget(self._hint)

    # ------------------------------------------------------------------ 채우기
    def set_mission_courses(self, names):
        self._mission_courses = ['(none)'] + list(names)
        self.refresh(self._courses, self.active_row())

    def refresh(self, courses, active_row=None):
        self._loading = True
        self._courses = list(courses)
        self._table.setRowCount(len(self._courses))
        for row, course in enumerate(self._courses):
            visible = QTableWidgetItem()
            visible.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
            visible.setCheckState(Qt.Checked if course.visible else Qt.Unchecked)
            self._table.setItem(row, self.COL_VISIBLE, visible)

            name = QTableWidgetItem(course.name + ('*' if course.dirty else ''))
            name.setIcon(_swatch(course.color))
            name.setToolTip(course.path)
            if course.dirty:
                name.setForeground(QColor(theme.COLOR_OK))
            self._table.setItem(row, self.COL_NAME, name)

            points = QTableWidgetItem(str(len(course)))
            points.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self._table.setItem(row, self.COL_POINTS, points)

            combo = QComboBox()
            combo.addItems(self._mission_courses)
            current = course.mission_course or '(none)'
            if current not in self._mission_courses:
                combo.addItem(current)
            combo.setCurrentText(current)
            combo.currentTextChanged.connect(
                lambda text, r=row: self._on_binding(r, text))
            self._table.setCellWidget(row, self.COL_COURSE, combo)

        if active_row is not None and 0 <= active_row < len(self._courses):
            self._table.selectRow(active_row)
        self._loading = False
        self._remove.setEnabled(bool(self._courses))

    def active_row(self):
        rows = self._table.selectionModel().selectedRows() if self._table.selectionModel() else []
        return rows[0].row() if rows else (0 if self._courses else None)

    # ------------------------------------------------------------------ 이벤트
    def _on_remove(self):
        row = self.active_row()
        if row is not None:
            self.remove_requested.emit(row)

    def _on_selection(self):
        if self._loading:
            return
        row = self.active_row()
        if row is not None:
            self.active_changed.emit(row)

    def _on_item_changed(self, item):
        if self._loading or item.column() != self.COL_VISIBLE:
            return
        self.visibility_changed.emit(item.row(), item.checkState() == Qt.Checked)

    def _on_binding(self, row, text):
        if self._loading:
            return
        self.binding_changed.emit(row, '' if text == '(none)' else text)
