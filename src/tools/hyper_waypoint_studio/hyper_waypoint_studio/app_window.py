#!/usr/bin/env python3
# =====================================================================
# 스튜디오 본체. 캔버스 하나 위에 코스/배경/라벨/차량을 겹쳐 놓고, 모드가 무엇을
# 만질 수 있는지를 정합니다.
#
# 파일 안전에 대한 약속이 이 파일의 핵심입니다.
#   - 파일을 여는 것은 언제나 읽기 전용입니다.
#   - 명시적인 저장 동작(코스 저장 / 다른 이름으로 / 미션 저장 / 정렬 저장) 말고는
#     아무것도 디스크에 쓰지 않습니다. 자동 저장도, 모드 전환 시 저장도 없습니다.
#   - 웨이포인트 CSV를 truncate하는 것은 이 프로그램 전체에서 레코더의 ~/start
#     하나뿐입니다. 그게 auto_start:=false가 존재하는 이유이고, 스튜디오가 코스를
#     잃는 두 번째 경로가 되어서는 안 됩니다.
# =====================================================================

import math
import os

from python_qt_binding.QtCore import Qt, QTimer, Signal
from python_qt_binding.QtGui import QColor, QKeySequence
from python_qt_binding.QtWidgets import (
    QAction, QActionGroup, QApplication, QCheckBox, QDialog, QDialogButtonBox,
    QDockWidget, QFileDialog, QLabel, QMainWindow, QMessageBox, QVBoxLayout,
    QWidget)

from . import formats, geometry, items, ros_link, theme
from .course_model import CourseModel
from .items import CourseItem, HeadingItem, LabelMarker, OverlayItem, VehicleItem, WaypointHandle
from .mission_model import MissionModel
from .panels.drive_panel import DrivePanel
from .panels.edit_panel import EditPanel
from .panels.layers_panel import LayersPanel
from .panels.overlay_panel import OverlayPanel
from .panels.record_panel import RecordPanel
from .scene import StudioScene, StudioView

HYPER = os.path.expanduser('~/HYPER')
WAYPOINT_DIR = os.path.join(HYPER, 'src/planning/hyper_waypoint/waypoints')
MISSION_DIR = os.path.join(HYPER, 'src/planning/hyper_planner/config')
COURSE_MESHES = os.path.join(
    HYPER, 'src/simulator/hyper_gazebo/worlds/models/driving_course/meshes')

MODES = ('view', 'edit', 'record', 'drive')
MODE_CAPTIONS = {'view': '보기', 'edit': '편집', 'record': '녹화', 'drive': '주행'}


class UnsavedDialog(QDialog):
    """닫기 전에 저장하지 않은 문서를 한 번에 보여 줍니다."""

    def __init__(self, parent, names):
        super().__init__(parent)
        self.setWindowTitle('저장하지 않은 변경')
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel('저장하지 않은 변경이 있습니다:'))
        self._boxes = []
        for name in names:
            box = QCheckBox(name)
            box.setChecked(True)
            layout.addWidget(box)
            self._boxes.append(box)
        buttons = QDialogButtonBox()
        self.save = buttons.addButton('선택 저장', QDialogButtonBox.AcceptRole)
        self.discard = buttons.addButton('저장 안 함', QDialogButtonBox.DestructiveRole)
        self.cancel = buttons.addButton('취소', QDialogButtonBox.RejectRole)
        self.save.clicked.connect(lambda: self.done(1))
        self.discard.clicked.connect(lambda: self.done(2))
        self.cancel.clicked.connect(lambda: self.done(0))
        layout.addWidget(buttons)

    def checked(self):
        return [i for i, box in enumerate(self._boxes) if box.isChecked()]


class StudioWindow(QMainWindow):

    def __init__(self, link, initial_mode='view', courses=(), mission=None,
                 overlay=None, destination=''):
        super().__init__()
        self.setWindowTitle('waypoint studio')
        self.setStyleSheet(theme.WINDOW_STYLE)
        self.resize(1500, 950)

        self._link = link
        self._mode = 'view'
        self._courses = []          # [CourseModel]
        self._layers = []           # [{'course': CourseItem, 'heading': HeadingItem}]
        self._handles = []          # 편집 중인 코스의 WaypointHandle
        self._labels = {}           # name -> LabelMarker
        self._mission = None
        self._overlay_item = None
        self._overlay_path = None
        self._overlay_alignable = False
        self._overlay_dirty = False
        self._active_row = None
        self._selected_point = None
        self._active_label = None
        self._color_cursor = 0
        self._recorder_file = ''
        self._previous_points = []

        self._scene = StudioScene()
        self._view = StudioView(self._scene)
        self.setCentralWidget(self._view)
        self._view.clicked_at.connect(self._on_canvas_click)
        self._view.cursor_moved.connect(self._on_cursor_moved)

        # 녹화 중인 경로와 미션이 보낸 경로. 코스 레이어와 구분되는 색으로 둡니다.
        self._live_path = CourseItem(theme.COLOR_PATH)
        self._live_path.set_dots_visible(False)
        self._live_path.setZValue(items.Z_LIVE_PATH)
        self._scene.addItem(self._live_path)
        self._mission_path = CourseItem(theme.COLOR_LABEL)
        self._mission_path.set_dots_visible(False)
        self._mission_path.setZValue(items.Z_LIVE_PATH)
        self._scene.addItem(self._mission_path)
        self._vehicle = VehicleItem()
        self._scene.addItem(self._vehicle)

        self._build_panels()
        self._build_menus()
        self._build_toolbar()
        self._wire_ros()
        self._link.start()

        self._status_label = QLabel('')
        self.statusBar().addPermanentWidget(self._status_label)

        for path in courses:
            self._add_course(path, frame=False)
        if mission:
            self._open_mission(mission)
        if overlay == 'gazebo':
            self._load_gazebo_overlay()
        elif overlay:
            self._load_image_overlay(overlay)
        if destination:
            self._record.set_destination(destination)
            self._sync_previous()

        self.set_mode(initial_mode if initial_mode in MODES else 'view')
        self._frame_all()
        self._refresh_layers()

        # hyper_rqt의 _poll과 같은 목적: 서비스가 아직 없으면 버튼을 꺼 두고,
        # 응답이 안 오는 future를 버립니다.
        self._poll_timer = QTimer(self)
        self._poll_timer.timeout.connect(self._poll)
        self._poll_timer.start(1000)

        # 레코더 status가 끊기면 stale 표시.
        self._stale_timer = QTimer(self)
        self._stale_timer.setSingleShot(True)
        self._stale_timer.timeout.connect(self._record.mark_stale)

    # ================================================================== 구성
    def _build_panels(self):
        self._layers_panel = LayersPanel()
        self._layers_panel.add_requested.connect(self._browse_course)
        self._layers_panel.remove_requested.connect(self._remove_course)
        self._layers_panel.active_changed.connect(self._set_active_row)
        self._layers_panel.visibility_changed.connect(self._set_visibility)
        self._layers_panel.binding_changed.connect(self._set_binding)

        self._overlay_panel = OverlayPanel()
        self._overlay_panel.load_gazebo.connect(self._load_gazebo_overlay)
        self._overlay_panel.load_image.connect(self._browse_overlay)
        self._overlay_panel.clear_overlay.connect(self._clear_overlay)
        self._overlay_panel.save_alignment.connect(self._save_alignment)
        self._overlay_panel.nudged.connect(self._nudge_overlay)
        self._overlay_panel.alpha_changed.connect(self._set_overlay_alpha)
        self._overlay_panel.fit_requested.connect(self._frame_all)

        self._edit = EditPanel()
        self._edit.label_selected.connect(self._select_label)
        self._edit.label_cleared.connect(self._clear_label)
        self._edit.save_mission.connect(self._save_mission)
        self._edit.save_course.connect(lambda: self._save_course(False))
        self._edit.save_course_as.connect(lambda: self._save_course(True))
        self._edit.undo.connect(self._undo)
        self._edit.redo.connect(self._redo)
        self._edit.flip_toggled.connect(self._toggle_flip)
        self._edit.recompute_headings.connect(self._recompute_headings)

        self._record = RecordPanel()
        self._record.start_requested.connect(self._start_recording)
        self._record.stop_requested.connect(self._stop_recording)
        self._record.browse_requested.connect(self._browse_destination)
        self._record.destination_changed.connect(lambda _: self._sync_previous())

        self._drive = DrivePanel()
        self._drive.goto_and_start.connect(
            lambda i, name: self._goto_step(i, name, then_start=True))
        self._drive.goto_only.connect(
            lambda i, name: self._goto_step(i, name, then_start=False))
        self._drive.teleport_requested.connect(self._teleport)
        self._drive.start.connect(lambda: self._call('start'))
        self._drive.cancel.connect(lambda: self._call('cancel'))
        self._drive.skip.connect(lambda: self._call('skip'))
        self._drive.restart.connect(lambda: self._call('restart'))

        self._docks = {}
        for key, caption, widget, area in (
                ('layers', '코스', self._layers_panel, Qt.LeftDockWidgetArea),
                ('overlay', '배경', self._overlay_panel, Qt.LeftDockWidgetArea),
                ('edit', '편집', self._edit, Qt.RightDockWidgetArea),
                ('record', '녹화', self._record, Qt.RightDockWidgetArea),
                ('drive', '주행', self._drive, Qt.RightDockWidgetArea)):
            dock = QDockWidget(caption, self)
            dock.setObjectName(f'dock_{key}')
            dock.setWidget(widget)
            dock.setFeatures(QDockWidget.DockWidgetMovable |
                             QDockWidget.DockWidgetFloatable)
            self.addDockWidget(area, dock)
            self._docks[key] = dock
        self.tabifyDockWidget(self._docks['edit'], self._docks['record'])
        self.tabifyDockWidget(self._docks['record'], self._docks['drive'])

    def _build_menus(self):
        file_menu = self.menuBar().addMenu('파일')
        for caption, slot, shortcut in (
                ('코스 추가…', self._browse_course, QKeySequence.Open),
                ('미션 열기…', self._browse_mission, None),
                ('배경 이미지…', self._browse_overlay, None)):
            action = QAction(caption, self)
            if shortcut:
                action.setShortcut(shortcut)
            action.triggered.connect(slot)
            file_menu.addAction(action)
        file_menu.addSeparator()
        save = QAction('코스 저장', self)
        save.setShortcut(QKeySequence.Save)
        save.triggered.connect(lambda: self._save_course(False))
        file_menu.addAction(save)
        save_as = QAction('코스를 다른 이름으로…', self)
        save_as.setShortcut(QKeySequence.SaveAs)
        save_as.triggered.connect(lambda: self._save_course(True))
        file_menu.addAction(save_as)
        save_mission = QAction('미션 저장', self)
        save_mission.triggered.connect(self._save_mission)
        file_menu.addAction(save_mission)
        file_menu.addSeparator()
        quit_action = QAction('끝내기', self)
        quit_action.setShortcut(QKeySequence.Quit)
        quit_action.triggered.connect(self.close)
        file_menu.addAction(quit_action)

        edit_menu = self.menuBar().addMenu('편집')
        undo = QAction('되돌리기', self)
        undo.setShortcut(QKeySequence.Undo)
        undo.triggered.connect(self._undo)
        redo = QAction('다시', self)
        redo.setShortcut(QKeySequence.Redo)
        redo.triggered.connect(self._redo)
        delete = QAction('선택한 점 삭제', self)
        delete.setShortcut(QKeySequence.Delete)
        delete.triggered.connect(self._delete_selected)
        for action in (undo, redo, delete):
            edit_menu.addAction(action)

        view_menu = self.menuBar().addMenu('보기')
        fit = QAction('화면 맞춤', self)
        fit.setShortcut('F')
        fit.triggered.connect(self._frame_all)
        view_menu.addAction(fit)
        grid = QAction('격자', self)
        grid.setCheckable(True)
        grid.setChecked(True)
        grid.toggled.connect(self._view.set_grid_visible)
        view_menu.addAction(grid)
        self._headings_action = QAction('헤딩 표시', self)
        self._headings_action.setCheckable(True)
        self._headings_action.setChecked(True)
        self._headings_action.toggled.connect(self._refresh_geometry)
        view_menu.addAction(self._headings_action)

    def _build_toolbar(self):
        bar = self.addToolBar('mode')
        bar.setMovable(False)
        group = QActionGroup(self)
        self._mode_actions = {}
        for mode in MODES:
            action = QAction(MODE_CAPTIONS[mode], self)
            action.setCheckable(True)
            action.triggered.connect(lambda _=False, m=mode: self.set_mode(m))
            group.addAction(action)
            bar.addAction(action)
            self._mode_actions[mode] = action

    def _wire_ros(self):
        link = self._link
        link.recorder_status.connect(self._on_recorder_status)
        link.recorder_path.connect(self._on_recorder_path)
        link.mission_status.connect(self._on_mission_status)
        link.mission_path.connect(self._on_mission_path)
        link.mission_steps.connect(self._drive.set_steps_from_node)
        link.vehicle_pose.connect(self._on_vehicle_pose)
        link.call_finished.connect(self._on_call_finished)
        if not link.available:
            self._record.set_disconnected(link.reason)
            self._drive.set_services_ready(False, link.reason)
        # 연결이 끝난 뒤에야 스핀을 시작합니다 -- latched 토픽의 첫 값을 놓치지
        # 않으려는 것입니다(ros_link.RosLink.start 주석).

    # ================================================================== 모드
    def set_mode(self, mode):
        if mode == self._mode:
            return
        # 녹화 중에 녹화 모드를 벗어나면 아무도 안 보는 레코더가 남습니다 --
        # 코스를 잃는 가장 흔한 경로라 막습니다.
        if self._mode == 'record' and self._record.recording and mode != 'record':
            QMessageBox.information(
                self, '녹화 중', '먼저 Stop을 눌러 녹화를 끝내세요.')
            self._mode_actions['record'].setChecked(True)
            return
        # 편집 모드로 들어가는 것은 녹화 중에는 막습니다: 편집하는 파일이 지금
        # truncate되는 중인 파일일 수 있습니다.
        if mode == 'edit' and self._record.recording:
            QMessageBox.information(
                self, '녹화 중', '녹화 중에는 편집할 수 없습니다.')
            self._mode_actions[self._mode].setChecked(True)
            return

        self._mode = mode
        self._mode_actions[mode].setChecked(True)
        self._docks['edit'].setVisible(mode == 'edit')
        self._docks['record'].setVisible(mode == 'record')
        self._docks['drive'].setVisible(mode == 'drive')
        if mode in ('edit', 'record', 'drive'):
            self._docks[mode].raise_()
        # 정렬은 보기/편집에서만. 주행을 보다가 방향키로 배경을 밀어 버리는 사고 방지.
        self._overlay_panel.setEnabled(mode in ('view', 'edit'))
        self._rebuild_handles()
        self._update_title()

    # ================================================================== 코스
    def _next_color(self):
        color = theme.COURSE_COLORS[self._color_cursor % len(theme.COURSE_COLORS)]
        self._color_cursor += 1
        return color

    def _browse_course(self):
        start = WAYPOINT_DIR if os.path.isdir(WAYPOINT_DIR) else os.path.expanduser('~')
        paths, _ = QFileDialog.getOpenFileNames(
            self, '웨이포인트 CSV 열기', start, 'CSV (*.csv);;모든 파일 (*)')
        for path in paths:
            self._add_course(path)

    def _add_course(self, path, frame=True):
        path = os.path.abspath(path)
        if any(os.path.abspath(c.path) == path for c in self._courses):
            self.statusBar().showMessage(f'{os.path.basename(path)}는 이미 열려 있습니다.', 4000)
            return
        try:
            course = CourseModel.load(path, self._next_color())
        except (OSError, ValueError) as exc:
            QMessageBox.warning(self, '열 수 없음', str(exc))
            return

        item = CourseItem(course.color)
        heading = HeadingItem(course.color)
        self._scene.addItem(item)
        self._scene.addItem(heading)
        self._courses.append(course)
        self._layers.append({'course': item, 'heading': heading})
        self._auto_bind(course)
        # 편집 대상은 먼저 연 코스로 둡니다 -- 보통 그게 main 코스이고, 갈래를
        # 추가로 올렸다고 편집 대상이 조용히 갈래로 옮겨 가면 안 됩니다.
        if self._active_row is None:
            self._active_row = 0
        self._refresh_geometry()
        self._refresh_layers()
        if frame:
            self._frame_all()

    def _auto_bind(self, course):
        """파일 이름으로 미션 코스를 짐작합니다. 틀리면 목록에서 바꾸면 됩니다."""
        if self._mission is None:
            return
        stem = os.path.splitext(course.name)[0]
        for name in self._mission.course_names:
            if name == 'main':
                continue
            entry = (self._mission.doc.get('courses') or {}).get(name) or {}
            if os.path.splitext(str(entry.get('csv', '')))[0] == stem:
                course.mission_course = name
                return
        # 갈래가 아니면서 아직 main이 없으면 main으로 둡니다.
        if not any(c.mission_course == 'main' for c in self._courses):
            course.mission_course = 'main'

    def _remove_course(self, row):
        if not 0 <= row < len(self._courses):
            return
        course = self._courses[row]
        if course.dirty:
            answer = QMessageBox.question(
                self, '저장하지 않은 변경',
                f'{course.name}에 저장하지 않은 변경이 있습니다. 그래도 뺄까요?',
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                return
        layer = self._layers.pop(row)
        self._scene.removeItem(layer['course'])
        self._scene.removeItem(layer['heading'])
        self._courses.pop(row)
        self._active_row = min(self._active_row or 0, len(self._courses) - 1)
        if not self._courses:
            self._active_row = None
        self._rebuild_handles()
        self._refresh_layers()

    def _set_active_row(self, row):
        if row == self._active_row:
            return
        self._active_row = row
        self._rebuild_handles()
        self._refresh_labels()
        self._update_title()

    def _set_visibility(self, row, visible):
        if not 0 <= row < len(self._courses):
            return
        self._courses[row].visible = visible
        self._layers[row]['course'].setVisible(visible)
        self._layers[row]['heading'].setVisible(
            visible and self._headings_action.isChecked())
        if row == self._active_row:
            self._rebuild_handles()

    def _set_binding(self, row, name):
        if not 0 <= row < len(self._courses):
            return
        self._courses[row].mission_course = name or None
        self._refresh_labels()

    @property
    def _active(self):
        if self._active_row is None or not 0 <= self._active_row < len(self._courses):
            return None
        return self._courses[self._active_row]

    # ================================================================== 그리기
    def _refresh_geometry(self):
        show_headings = self._headings_action.isChecked()
        for course, layer in zip(self._courses, self._layers):
            layer['course'].set_points(course.xs, course.ys)
            layer['course'].setVisible(course.visible)
            if show_headings:
                # 점이 많으면 헤딩을 솎아 그립니다 -- 다 그리면 선이 뭉개집니다.
                stride = max(1, len(course) // 400)
                layer['heading'].set_headings(
                    course.xs, course.ys, course.yaws, stride=stride)
            layer['heading'].setVisible(course.visible and show_headings)

    def _rebuild_handles(self):
        """편집 중인 코스에만 핸들을 만듭니다.

        1500점짜리 코스 여러 개를 항상 아이템으로 두면 못 씁니다. 다른 모드에서는
        CourseItem의 점 표시만으로 충분합니다.
        """
        for handle in self._handles:
            self._scene.removeItem(handle)
        self._handles = []
        course = self._active
        if self._mode != 'edit' or course is None or not course.visible:
            return
        for i, (x, y) in enumerate(zip(course.xs, course.ys)):
            handle = WaypointHandle(i, x, y, course.color,
                                    self._on_handle_moved, self._on_handle_clicked)
            handle.set_reverse(course.flips[i] < 0)
            self._scene.addItem(handle)
            self._handles.append(handle)

    def _refresh_layers(self):
        if self._mission is not None:
            self._layers_panel.set_mission_courses(self._mission.course_names)
        self._layers_panel.refresh(self._courses, self._active_row)
        self._refresh_labels()
        self._edit.set_undo_state(
            bool(self._active and self._active.can_undo),
            bool(self._active and self._active.can_redo))
        course = self._active
        if course is not None and len(course):
            import math
            self._edit.set_heading_offset(
                math.degrees(course.heading_offset()), len(course))
        else:
            self._edit.set_heading_offset(0.0, 0)
        self._update_title()

    def _frame_all(self):
        """코스와 배경이 모두 보이도록 맞춥니다.

        itemsBoundingRect를 쓰지 않는 이유가 두 가지입니다. 라벨/핸들은
        ItemIgnoresTransformations라 경계가 픽셀 단위인데 씬 단위로 섞여 들어와
        (라벨 하나가 200 m처럼) 범위를 부풀리고, 변환이 걸린 배경 아이템은 자동
        경계에 제대로 안 잡혀 잘못 놓인 배경이 화면 밖에 있다는 힌트조차 없습니다.
        """
        from python_qt_binding.QtCore import QRectF

        rect = None
        for course in self._courses:
            if not course.visible or not len(course):
                continue
            bounds = QRectF(min(course.xs), min(course.ys), 0, 0)
            bounds.setRight(max(course.xs))
            bounds.setBottom(max(course.ys))
            rect = bounds if rect is None else rect.united(bounds)
        if self._overlay_item is not None:
            overlay = self._overlay_item.map_rect()
            rect = overlay if rect is None else rect.united(overlay)
        if rect is None:
            return
        self._view.frame(rect.normalized())

    # ================================================================== 편집
    def _on_handle_clicked(self, index):
        self._selected_point = index
        course = self._active
        if course is not None and 0 <= index < len(course):
            self._edit.set_selected_point(
                index, course.xs[index], course.ys[index], course.yaws[index],
                course.flips[index] < 0)

    def _on_handle_moved(self, index, x, y):
        course = self._active
        if course is None:
            return
        course.move_point(index, x, y)
        self._refresh_geometry()
        self._sync_handles(index)
        self._on_handle_clicked(index)
        self._refresh_labels()
        self._check_reverse(index)
        self._refresh_layers()

    def _sync_handles(self, around):
        """모델이 바꾼 좌표를 핸들 위치에 되돌립니다(이웃 yaw까지 다시 구해지므로)."""
        course = self._active
        if course is None:
            return
        for i in (around - 1, around, around + 1):
            if 0 <= i < len(self._handles) and i < len(course):
                handle = self._handles[i]
                handle.blockSignals(True)
                handle.setPos(course.xs[i], course.ys[i])
                handle.set_reverse(course.flips[i] < 0)
                handle.blockSignals(False)

    def _check_reverse(self, index):
        """편집한 자리가 미션의 reverse 스텝 안이면 로더와 같은 검사를 미리 돌립니다."""
        course = self._active
        if course is None or self._mission is None:
            self._edit.set_reverse_note('')
            return
        # 지금 편집 중인 코스에 지정된 미션 코스의 스텝만 봅니다. 라벨이 코스마다
        # 독립이라, 다른 코스의 스텝을 여기 대고 재면 엉뚱한 구간이 걸립니다.
        bound = course.mission_course or 'main'
        steps = list(self._mission.doc.get('steps') or [])
        for route in (self._mission.doc.get('routes') or {}).values():
            steps.extend(route or [])
        for step in steps:
            if not isinstance(step, dict) or step.get('type') != 'drive':
                continue
            if (step.get('course') or 'main') != bound:
                continue
            label = step.get('until')
            point = self._mission.position(bound, label) if label else None
            if point is None:
                continue
            x, y = point
            end, _ = course.nearest(x, y)
            start = max(0, end - 60)
            if not start <= index <= end:
                continue
            fraction = course.reverse_fraction(start, end)
            expected = bool(step.get('reverse'))
            bad = (expected and fraction < 0.5) or (not expected and fraction > 0.5)
            self._edit.set_reverse_note(
                f"'{label}' 구간 후진 비율 {fraction:.0%} "
                f"(reverse: {'true' if expected else 'false'})"
                + ('  ** 로드 시 경고가 납니다 **' if bad else ''), bad)
            return
        self._edit.set_reverse_note('')

    def _delete_selected(self):
        course = self._active
        if self._mode != 'edit' or course is None or self._selected_point is None:
            return
        if not course.delete_point(self._selected_point):
            self.statusBar().showMessage('더 지울 수 없습니다 (점이 2개 이하).', 4000)
            return
        self._selected_point = None
        self._edit.set_selected_point(None)
        self._refresh_geometry()
        self._rebuild_handles()
        self._refresh_layers()

    def _toggle_flip(self):
        course = self._active
        if course is None or self._selected_point is None:
            return
        index = self._selected_point
        course.set_flip(index, -course.flips[index])
        self._refresh_geometry()
        self._sync_handles(index)
        self._on_handle_clicked(index)
        self._refresh_layers()

    def _recompute_headings(self):
        """코스 전체의 헤딩을 경로에서 다시 구합니다.

        확인을 받는 이유: 전진/후진 표시(flips)는 녹화된 yaw에서 뽑은 값이라, 틀어진
        각이 90도 근처면 그 판정 자체가 못 믿을 값이 됩니다. 사람이 후진 점 개수와
        전환점 개수를 보고 "이 코스가 정말 그렇게 생겼나"를 판단해야 합니다.
        """
        course = self._active
        if course is None:
            return
        import math

        offset = math.degrees(course.heading_offset())
        reverse = sum(1 for f in course.flips if f < 0)
        answer = QMessageBox.question(
            self, '헤딩 다시 계산',
            f'{course.name}의 헤딩 {len(course)}개를 경로 진행 방향에서 다시 구합니다.\n\n'
            f'  경로 대비 현재 중앙값 : {offset:+.1f}°\n'
            f'  후진으로 판정된 점    : {reverse}개\n\n'
            '전진/후진 표시는 그대로 두고 방향만 다시 구합니다.\n'
            '위 후진 점 개수가 이 코스의 실제 모양과 맞는지 먼저 확인하세요 -- '
            '그 판정은 지금의 (틀어진) yaw에서 뽑은 값입니다.\n\n'
            '되돌리기(Ctrl-Z)로 되돌릴 수 있고, 저장하기 전에는 파일이 바뀌지 않습니다.',
            QMessageBox.Ok | QMessageBox.Cancel, QMessageBox.Cancel)
        if answer != QMessageBox.Ok:
            return

        changed, largest = course.recompute_headings()
        self._refresh_geometry()
        self._rebuild_handles()
        self._refresh_layers()
        self.statusBar().showMessage(
            f'{course.name}: 헤딩 {changed}개 갱신, 최대 변화 '
            f'{math.degrees(largest):.1f}°', 8000)

    def _undo(self):
        course = self._active
        if course is not None and course.undo():
            self._refresh_geometry()
            self._rebuild_handles()
            self._refresh_layers()

    def _redo(self):
        course = self._active
        if course is not None and course.redo():
            self._refresh_geometry()
            self._rebuild_handles()
            self._refresh_layers()

    def _on_canvas_click(self, x, y, button):
        if self._mode != 'edit':
            return
        modifiers = QApplication.keyboardModifiers()
        course = self._active
        if course is None:
            return
        if modifiers & Qt.ShiftModifier and button == int(Qt.LeftButton):
            index, _ = course.nearest(x, y)
            new_index = course.insert_point(index, x, y)
            self._refresh_geometry()
            self._rebuild_handles()
            self._selected_point = new_index
            self._on_handle_clicked(new_index)
            self._refresh_layers()
            return
        # 라벨이 골라져 있으면 클릭이 곧 그 라벨의 배치입니다. 스냅 대상은 편집 중인
        # 코스가 아니라 그 라벨이 속한 코스입니다 -- 갈래 라벨을 고른 채 main을 클릭했다고
        # 라벨이 main으로 옮겨 붙으면 안 됩니다.
        if self._active_label and self._mission is not None:
            label_course, name = formats.split_key(self._active_label)
            target = self._course_for(label_course)
            if target is None:
                self._edit.set_place_hint(
                    f"'{name}'은 코스 '{label_course}'의 라벨입니다. 그 CSV를 열어야 "
                    f"배치할 수 있습니다.")
                return
            index, distance = target.nearest(x, y)
            self._mission.place(label_course, name, target.xs[index], target.ys[index])
            self._edit.set_place_hint(
                f'{name} [{label_course}] -> wp #{index}, 스냅 {distance:.2f} m'
                + ('   ** 경로에서 먼 클릭입니다 **' if distance > 2.0 else ''))
            self._refresh_labels()
            self._update_title()

    def _on_cursor_moved(self, x, y):
        self._status_label.setText(
            f'x {x:8.2f}   y {y:8.2f} m      {self._view.meters_per_pixel():.3f} m/px')

    # ================================================================== 라벨
    def _browse_mission(self):
        start = MISSION_DIR if os.path.isdir(MISSION_DIR) else os.path.expanduser('~')
        path, _ = QFileDialog.getOpenFileName(
            self, '미션 YAML 열기', start, 'YAML (*.yaml *.yml);;모든 파일 (*)')
        if path:
            self._open_mission(path)

    def _open_mission(self, path):
        if self._mission is not None and self._mission.dirty:
            answer = QMessageBox.question(
                self, '저장하지 않은 라벨',
                f'{self._mission.name}의 라벨이 저장되지 않았습니다. 버릴까요?',
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                return
        try:
            self._mission = MissionModel.load(path)
        except Exception as exc:              # noqa: BLE001 -- 이유를 보여 줍니다
            QMessageBox.warning(self, '미션을 열 수 없음', str(exc))
            return
        for course in self._courses:
            if course.mission_course is None:
                self._auto_bind(course)
        self._drive.set_steps_from_mission(self._mission)
        self._refresh_layers()

    def _course_for(self, name):
        for course in self._courses:
            if course.mission_course == name:
                return course
        return None

    def _bound_courses(self):
        """{미션 코스 이름: 그 코스로 지정된 CourseItem 또는 None}.

        라벨은 자기 코스의 CSV에만 스냅되므로(mission_loader.snap_labels), 스냅 거리도
        저장도 코스마다 따로 봐야 합니다. 안 연 코스는 None -- 그 코스의 라벨은 거리를
        못 재지만 저장할 때 좌표는 그대로 다시 쓰입니다.
        """
        if self._mission is None:
            return {}
        return {name: self._course_for(name) for name in self._mission.course_names}

    def _refresh_labels(self):
        for marker in self._labels.values():
            self._scene.removeItem(marker)
        self._labels = {}
        if self._mission is None:
            self._edit.set_labels([], None)
            return

        report = self._mission.snap_report(self._bound_courses())
        for course, name, _index, _distance, state in report:
            if state in ('sentinel', 'missing'):
                continue          # 찍을 좌표가 없습니다.
            key = formats.label_key(course, name)
            x, y = self._mission.position(course, name)
            # 화면 글자에는 main이 아닐 때만 코스를 붙입니다. 같은 이름이 두 갈래에
            # 있으면(t_left_end / t_right_end 같은 짝) 어느 쪽인지 보여야 합니다.
            text = name if course == 'main' else f'{name} [{course}]'
            marker = LabelMarker(key, text, x, y,
                                 self._on_label_moved, self._select_label)
            marker.set_state(state if state != 'nocourse' else 'ok')
            marker.set_active(key == self._active_label)
            self._scene.addItem(marker)
            self._labels[key] = marker
        self._edit.set_labels(report, self._mission.name, self._active_label,
                              self._mission.orphans())

    def _select_label(self, key):
        self._active_label = key
        for marker_key, marker in self._labels.items():
            marker.set_active(marker_key == key)
        course, name = formats.split_key(key)
        if self._course_for(course) is None:
            self._edit.set_place_hint(
                f"'{name}' 선택됨 -- 코스 '{course}'가 안 열려 있어 배치할 수 없습니다. "
                f"그 CSV를 먼저 여세요.")
        else:
            self._edit.set_place_hint(
                f"'{name}' 선택됨 -- '{course}' 코스를 클릭하면 그 자리로 옮깁니다.")

    def _clear_label(self, key):
        if self._mission is None:
            return
        course, name = formats.split_key(key)
        self._mission.remove(course, name)
        if self._active_label == key:
            self._active_label = None
        self._refresh_labels()
        self._update_title()

    def _on_label_moved(self, key, x, y):
        if self._mission is None:
            return
        label_course, name = formats.split_key(key)
        # 끌어 놓은 라벨은 언제나 **자기 코스**에 스냅합니다. 지금 편집 중인 코스가
        # 아니라 -- 그러면 갈래 라벨이 조용히 main 위로 옮겨 붙습니다.
        course = self._course_for(label_course)
        if course is not None:
            index, distance = course.nearest(x, y)
            self._mission.place(label_course, name, course.xs[index], course.ys[index])
            self._edit.set_place_hint(
                f'{name} [{label_course}] -> wp #{index}, 스냅 {distance:.2f} m')
        else:
            self._mission.place(label_course, name, x, y)
            self._edit.set_place_hint(
                f"{name}: 코스 '{label_course}'가 안 열려 있어 클릭한 좌표 그대로 뒀습니다.")
        self._refresh_labels()
        self._update_title()

    # ================================================================== 배경
    def _load_gazebo_overlay(self):
        texture = os.path.join(COURSE_MESHES, 'course.png')
        quad = os.path.join(COURSE_MESHES, 'ground.obj')
        for required in (texture, quad):
            if not os.path.exists(required):
                QMessageBox.warning(self, '배경 없음', f"'{required}'가 없습니다.")
                return
        try:
            extent = formats.read_obj_extent(quad)
            data, width, height = formats.load_background(texture)
        except Exception as exc:              # noqa: BLE001
            QMessageBox.warning(self, '배경을 못 읽었습니다', str(exc))
            return
        self._install_overlay(texture, data, width, height)
        self._overlay_item.set_extent(extent)
        self._overlay_alignable = False
        self._overlay_dirty = False
        self._overlay_panel.set_overlay(texture, alignable=False)
        self._frame_all()

    def _browse_overlay(self):
        start = COURSE_MESHES if os.path.isdir(COURSE_MESHES) else os.path.expanduser('~')
        path, _ = QFileDialog.getOpenFileName(
            self, '배경 이미지 열기', start,
            '이미지 (*.png *.jpg *.jpeg *.tif *.tiff);;모든 파일 (*)')
        if path:
            self._load_image_overlay(path)

    def _load_image_overlay(self, path):
        try:
            data, width, height = formats.load_background(path)
        except Exception as exc:              # noqa: BLE001
            QMessageBox.warning(self, '배경을 못 읽었습니다', str(exc))
            return
        xs = [x for course in self._courses for x in course.xs]
        ys = [y for course in self._courses for y in course.ys]
        cx, cy, width_m, rot = formats.load_alignment(path, xs, ys)
        self._install_overlay(path, data, width, height)
        self._overlay_item.set_alignment(cx, cy, width_m, rot)
        self._overlay_alignable = True
        self._overlay_dirty = not os.path.exists(formats.alignment_path(path))
        self._push_overlay_readout()
        self._frame_all()

    def _install_overlay(self, path, data, width, height):
        if self._overlay_item is not None:
            self._scene.removeItem(self._overlay_item)
        self._overlay_item = OverlayItem(data, width, height)
        self._scene.addItem(self._overlay_item)
        self._overlay_path = path

    def _clear_overlay(self):
        if self._overlay_item is not None:
            self._scene.removeItem(self._overlay_item)
        self._overlay_item = None
        self._overlay_path = None
        self._overlay_alignable = False
        self._overlay_dirty = False
        self._overlay_panel.set_overlay(None, alignable=False)

    def _nudge_overlay(self, dx, dy, scale, rot):
        if self._overlay_item is None or not self._overlay_alignable:
            return
        self._overlay_item.nudge(dx, dy, scale, rot)
        self._overlay_dirty = True
        self._push_overlay_readout()

    def _set_overlay_alpha(self, alpha):
        if self._overlay_item is not None:
            self._overlay_item.setOpacity(alpha)

    def _push_overlay_readout(self):
        item = self._overlay_item
        self._overlay_panel.set_overlay(
            self._overlay_path, self._overlay_alignable,
            item.cx, item.cy, item.width_m, item.rot_deg, self._overlay_dirty)

    def _save_alignment(self):
        if self._overlay_item is None or not self._overlay_alignable:
            return
        item = self._overlay_item
        try:
            path = formats.save_alignment(
                self._overlay_path, item.cx, item.cy, item.width_m, item.rot_deg)
        except OSError as exc:
            QMessageBox.warning(self, '정렬 저장 실패', str(exc))
            return
        self._overlay_dirty = False
        self._push_overlay_readout()
        self.statusBar().showMessage(f'{os.path.basename(path)} 저장됨', 5000)

    # ================================================================== 저장
    def _save_course(self, as_new):
        course = self._active
        if course is None:
            return
        target = course.path
        if as_new:
            default_dir = os.path.join(WAYPOINT_DIR, 'track')
            os.makedirs(default_dir, exist_ok=True) if os.path.isdir(WAYPOINT_DIR) else None
            target, _ = QFileDialog.getSaveFileName(
                self, '코스를 다른 이름으로 저장',
                os.path.join(default_dir, course.name), 'CSV (*.csv)')
            if not target:
                return
        elif self._record.recording and os.path.abspath(target) == os.path.abspath(
                self._recorder_file or ''):
            QMessageBox.warning(
                self, '녹화 중', '레코더가 지금 이 파일에 쓰고 있습니다.')
            return
        try:
            course.save(target)
        except OSError as exc:
            QMessageBox.warning(self, '저장 실패', str(exc))
            return
        self.statusBar().showMessage(f'{os.path.basename(target)} 저장됨', 5000)
        self._refresh_layers()

    def _save_mission(self):
        if self._mission is None:
            return
        courses = self._bound_courses()
        report = self._mission.snap_report(courses)
        over = [entry for entry in report if entry[4] == 'over']
        missing = [entry for entry in report if entry[4] == 'missing']
        if over or missing:
            lines = []
            for course, name, _i, distance, _s in over:
                lines.append(f'  {name} [{course}]: {distance:.2f} m '
                             f'(허용 {self._mission.snap_tolerance:.2f} m)')
            for course, name, _i, _d, _s in missing:
                lines.append(f'  {name} [{course}]: 미배치')
            answer = QMessageBox.warning(
                self, '이대로 저장하면 미션이 로드되지 않습니다',
                'mission_manager는 라벨이 허용 오차를 넘으면 미션 전체를 거부합니다.\n\n'
                + '\n'.join(lines) + '\n\n그래도 저장할까요?',
                QMessageBox.Save | QMessageBox.Cancel, QMessageBox.Cancel)
            if answer != QMessageBox.Save:
                return
        try:
            self._mission.save(courses)
        except formats.FileChangedError:
            answer = QMessageBox.question(
                self, '파일이 바뀌었습니다',
                f'{self._mission.name}가 연 뒤에 디스크에서 바뀌었습니다.\n'
                '다시 읽으면 지금 편집한 라벨을 잃습니다. 다시 읽을까요?',
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer == QMessageBox.Yes:
                self._mission.reload()
                self._refresh_labels()
            return
        except OSError as exc:
            QMessageBox.warning(self, '저장 실패', str(exc))
            return
        self.statusBar().showMessage(f'{self._mission.name} 저장됨', 5000)
        self._update_title()

    def _update_title(self):
        parts = ['waypoint studio']
        if self._mission is not None:
            parts.append(self._mission.name + ('*' if self._mission.dirty else ''))
        dirty = sum(1 for course in self._courses if course.dirty)
        if self._courses:
            note = f'{len(self._courses)} courses'
            if dirty:
                note += f' ({dirty} modified)'
            parts.append(note)
        parts.append(MODE_CAPTIONS[self._mode])
        self.setWindowTitle(' — '.join(parts))

    # ================================================================== 녹화
    def _browse_destination(self):
        start = WAYPOINT_DIR if os.path.isdir(WAYPOINT_DIR) else os.path.expanduser('~')
        path, _ = QFileDialog.getSaveFileName(
            self, '녹화할 CSV 고르기', start, 'CSV (*.csv)')
        if path:
            self._record.set_destination(path)
            self._sync_previous()

    def _sync_previous(self):
        """저장 파일 칸이 가리키는 CSV를 회색으로 깔아 둡니다.

        Record가 무엇을 덮어쓰는지 누르기 전에 보이게 하려는 것입니다
        (waypoint_record_gui.py가 미니맵에 하던 일과 같습니다).
        """
        if self._record.recording:
            return
        path = self._record.destination() or self._recorder_file
        if not path or not os.path.isfile(path):
            self._live_path.set_points([], [])
            return
        try:
            course = formats.read_course(path)
        except (OSError, ValueError):
            self._live_path.set_points([], [])
            return
        self._live_path.set_color(theme.COLOR_PREV)
        self._live_path.set_points(course.xs, course.ys)
        self._record.show_message(
            f'이전 녹화본 {len(course)} 점 -- Record를 누르면 덮어씁니다', theme.COLOR_OK)

    def _start_recording(self):
        destination = self._record.destination()
        if destination:
            # 열려 있는 코스를 덮어쓰려는 것이면 막습니다. 저장 안 한 편집이 있으면
            # 되돌릴 방법이 없습니다.
            for course in self._courses:
                if os.path.abspath(course.path) != os.path.abspath(destination):
                    continue
                if course.dirty:
                    QMessageBox.warning(
                        self, '덮어쓸 수 없음',
                        f'{course.name}에 저장하지 않은 편집이 있는데 녹화가 이 파일을 '
                        '지웁니다. 먼저 저장하거나 다른 이름을 고르세요.')
                    return
                answer = QMessageBox.question(
                    self, '열려 있는 코스입니다',
                    f'{course.name}는 지금 화면에 올라와 있습니다. '
                    '녹화를 시작하면 이 파일이 지워집니다. 계속할까요?',
                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
                if answer != QMessageBox.Yes:
                    return

            def applied(ok, message):
                if ok:
                    self._link.call(f'{self._link.recorder_ns}/start')
                else:
                    self._record.show_message(f'파일 이름 설정 실패: {message}',
                                              theme.COLOR_BAD)

            self._link.set_parameters(
                self._link.recorder_ns, {'output_csv': destination}, applied)
            return
        self._link.call(f'{self._link.recorder_ns}/start')

    def _stop_recording(self):
        self._link.call(f'{self._link.recorder_ns}/stop')

    # ================================================================== 주행
    def _goto_step(self, index, label, then_start):
        """스텝을 고른 뒤(필요하면) 출발합니다.

        인자를 파라미터로 밀어 넣고 인자 없는 Trigger를 부르는 방식은 이 스택의
        관례입니다(teleport_service의 label, model_service의 이름).
        step_label을 같이 보내는 이유: 스텝을 넣고 빼면 인덱스는 밀리지만 라벨은
        그대로라, 노드 쪽에서 라벨을 우선합니다.
        """
        values = {'step_index': int(index), 'step_label': label or ''}

        def applied(ok, message):
            if not ok:
                self._drive.show_message(f'스텝 설정 실패: {message}', theme.COLOR_BAD)
                return
            self._link.call(f'{self._link.manager_ns}/goto_step')
            if then_start:
                self._pending_start = True

        self._pending_start = False
        self._link.set_parameters(self._link.manager_ns, values, applied)

    def _teleport(self, label, offset_m):
        def applied(ok, message):
            if not ok:
                self._drive.show_message(f'순간이동 설정 실패: {message}', theme.COLOR_BAD)
                return
            self._link.call(f'{self._link.teleport_ns}/teleport')

        self._link.set_parameters(
            self._link.teleport_ns, {'label': label, 'offset_m': float(offset_m)},
            applied)

    def _call(self, verb):
        self._link.call(f'{self._link.manager_ns}/{verb}')

    # ================================================================== ROS
    def _on_recorder_status(self, status):
        self._stale_timer.start(int(theme.STALE_S * 1000))
        was_recording = self._record.recording
        self._record.update_status(status, ros_link.as_float)
        self._recorder_file = status.get('file', '')
        if self._record.recording:
            if not was_recording:
                # 녹화가 시작된 순간 파일은 truncate됐습니다. 깔아 둔 회색 선은 더
                # 이상 그 파일의 내용이 아닙니다.
                self._live_path.set_color(theme.COLOR_PATH)
                self._live_path.set_points([], [])
        elif was_recording:
            self._sync_previous()

    def _on_recorder_path(self, points):
        if not self._record.recording:
            return
        self._live_path.set_color(theme.COLOR_PATH)
        self._live_path.set_points([p[0] for p in points], [p[1] for p in points])

    def _on_mission_status(self, text):
        self._drive.set_status(text)
        if getattr(self, '_pending_start', False):
            # goto_step이 idle로 돌려놓은 뒤에 출발합니다.
            self._pending_start = False
            self._link.call(f'{self._link.manager_ns}/start')

    def _on_mission_path(self, points):
        self._mission_path.set_points([p[0] for p in points], [p[1] for p in points])

    def _on_vehicle_pose(self, x, y, yaw):
        self._vehicle.set_pose(x, y, yaw)
        self._vehicle.set_stale(False)

    def _on_call_finished(self, name, ok, message):
        target = self._record if name.startswith(self._link.recorder_ns) else self._drive
        target.show_message(
            f'{os.path.basename(name)}: {message}',
            theme.COLOR_GOOD if ok else theme.COLOR_BAD)

    def _poll(self):
        if not self._link.available:
            return
        self._link.drop_stale_calls()
        ready = self._link.service_ready(f'{self._link.manager_ns}/start')
        self._drive.set_services_ready(
            ready, 'mission_manager 서비스가 아직 없습니다')

    # ================================================================== 종료
    def _dirty_documents(self):
        documents = []
        for course in self._courses:
            if course.dirty:
                documents.append((course.name, lambda c=course: c.save()))
        if self._mission is not None and self._mission.dirty:
            documents.append((
                self._mission.name,
                lambda: self._mission.save(self._course_for('main') or self._active)))
        if self._overlay_dirty and self._overlay_alignable:
            item = self._overlay_item
            documents.append((
                os.path.basename(formats.alignment_path(self._overlay_path)),
                lambda: formats.save_alignment(
                    self._overlay_path, item.cx, item.cy, item.width_m, item.rot_deg)))
        return documents

    def closeEvent(self, event):
        documents = self._dirty_documents()
        if documents:
            dialog = UnsavedDialog(self, [name for name, _ in documents])
            result = dialog.exec_()
            if result == 0:
                event.ignore()
                return
            if result == 1:
                for index in dialog.checked():
                    try:
                        documents[index][1]()
                    except Exception as exc:      # noqa: BLE001
                        QMessageBox.warning(self, '저장 실패', str(exc))
                        event.ignore()
                        return
        self._link.shutdown()
        event.accept()
