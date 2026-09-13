#!/usr/bin/env python3
# =====================================================================
# 캔버스. 씬 좌표가 곧 map 프레임 미터입니다.
#
# 뷰가 scale(1, -1)을 걸어 y-up을 만듭니다 -- map은 X=East, Y=North인데 화면은 y가
# 아래로 증가하기 때문입니다. 이 뒤집기 때문에 글자/핸들은 전부
# ItemIgnoresTransformations여야 합니다(items.py 주석).
# =====================================================================

import math

from python_qt_binding.QtCore import QPointF, QRectF, Qt, Signal
from python_qt_binding.QtGui import QColor, QPainter, QPen, QTransform
from python_qt_binding.QtWidgets import QGraphicsScene, QGraphicsView

from . import theme

# 한 번에 볼 수 있는 범위. 너무 넓게 줌아웃하면 아무것도 안 보이고, 너무 좁으면
# 핸들 하나가 화면을 채웁니다.
MIN_SCALE = 0.05      # 픽셀당 20 m
MAX_SCALE = 200.0     # 픽셀당 0.5 cm

# 뷰가 쓰는 sceneRect의 반지름(m). 코스는 100 m대이고 map 프레임은 datum 기준이라
# 이 안을 벗어나지 않습니다. 자동 sceneRect(itemsBoundingRect)를 쓰지 않는 이유는
# StudioView.__init__의 setSceneRect 주석에 있습니다.
SCENE_RADIUS_M = 5000.0


class StudioScene(QGraphicsScene):
    """아이템만 담습니다. 격자와 축은 뷰가 배경으로 직접 그립니다."""

    def __init__(self):
        super().__init__()
        self.setBackgroundBrush(QColor(theme.COLOR_BG))


class StudioView(QGraphicsView):
    """map 프레임 캔버스.

    좌클릭 드래그는 아이템 조작(편집 모드) 또는 고무줄 선택, 가운데 버튼 드래그는
    팬, 휠은 커서 기준 줌입니다. 팬을 가운데 버튼에 둔 이유는 좌클릭이 편집
    모드에서 이미 "점을 끈다"에 쓰이기 때문입니다.
    """

    clicked_at = Signal(float, float, int)   # x, y, Qt 버튼
    cursor_moved = Signal(float, float)
    follow_released = Signal()               # 팬으로 차량 고정이 풀렸습니다

    # 서 있는 차의 odom 잡음으로 화면 전체를 다시 그리지 않기 위한 여유. 뷰포트
    # 전체 repaint는 배경 이미지가 깔린 코스에서 비쌉니다.
    FOLLOW_DEADZONE_PX = 1.5
    FOLLOW_DEADZONE_RAD = math.radians(1.0)

    def __init__(self, scene):
        super().__init__(scene)
        self.setRenderHint(QPainter.Antialiasing, True)
        self.setDragMode(QGraphicsView.RubberBandDrag)
        self.setTransformationAnchor(QGraphicsView.NoAnchor)
        self.setResizeAnchor(QGraphicsView.NoAnchor)
        self.setViewportUpdateMode(QGraphicsView.SmartViewportUpdate)
        self.setMouseTracking(True)
        self.setFrameStyle(0)
        # sceneRect를 넉넉한 고정 사각형으로 못박습니다. 기본값(itemsBoundingRect)은
        # 두 가지를 망가뜨립니다.
        #   1. 라벨/핸들이 ItemIgnoresTransformations라 그 경계가 실제 map 범위와
        #      다릅니다(_frame_all 주석과 같은 이유).
        #   2. 씬이 뷰포트보다 작은 축에서는 Qt가 정렬(AlignCenter)로 화면을 가운데
        #      못박고 뷰 변환의 이동을 통째로 무시합니다 -- 차량 고정이 그 축에서만
        #      조용히 안 먹었습니다. 팬도 코스 경계에서 걸렸습니다.
        self.setSceneRect(-SCENE_RADIUS_M, -SCENE_RADIUS_M,
                          2 * SCENE_RADIUS_M, 2 * SCENE_RADIUS_M)
        # 그 사각형은 늘 뷰포트보다 크므로, 그냥 두면 스크롤바가 영영 떠 있습니다.
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        # y-up. 이 한 줄이 이 파일의 거의 모든 주의사항의 원인입니다.
        self.scale(1.0, -1.0)
        self._panning = False
        self._pan_from = None
        self._grid_visible = True
        self._follow = False
        self._follow_point = None
        self._follow_yaw = 0.0
        self._applied_yaw = 0.0

    # ------------------------------------------------------------------ 줌/팬
    def wheelEvent(self, event):
        factor = 1.2 if event.angleDelta().y() > 0 else 1.0 / 1.2
        current = self._current_scale()
        if not (MIN_SCALE <= current * factor <= MAX_SCALE):
            return
        # 차량 고정 중에는 커서가 아니라 차량을 기준으로 줌합니다 -- 커서 기준으로
        # 줌하면 차가 화면 밖으로 밀렸다가 다음 pose에서 튕겨 돌아옵니다.
        if self._follow and self._follow_point is not None:
            self._applied_yaw = self._follow_yaw
            self._apply_view_transform(current * factor, self._follow_yaw - math.pi / 2,
                                       *self._follow_point)
            return
        # 커서 아래 지점이 제자리에 있도록 줌합니다.
        anchor = self.mapToScene(event.pos())
        self.scale(factor, factor)
        moved = self.mapToScene(event.pos())
        self.translate(moved.x() - anchor.x(), moved.y() - anchor.y())

    def mousePressEvent(self, event):
        if event.button() == Qt.MiddleButton:
            # 팬은 고정을 풉니다. 고정한 채로 팬을 무시하면 화면이 죽은 것처럼
            # 보이고, 왜 안 움직이는지 알 방법이 없습니다.
            if self._follow:
                self._follow = False
                self._derotate()
                self.follow_released.emit()
            self._panning = True
            self._pan_from = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
            return
        point = self.mapToScene(event.pos())
        # 아이템이 먼저 먹는 경우(핸들 드래그)에도 패널이 좌표를 알아야 하므로 먼저 냅니다.
        self.clicked_at.emit(point.x(), point.y(), int(event.button()))
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._panning and self._pan_from is not None:
            delta = event.pos() - self._pan_from
            self._pan_from = event.pos()
            # 씬 단위로 옮깁니다. y 뒤집기 때문에 dy 부호가 반대입니다.
            scale_x = self.transform().m11()
            scale_y = self.transform().m22()
            self.translate(delta.x() / scale_x, delta.y() / scale_y)
            event.accept()
            return
        point = self.mapToScene(event.pos())
        self.cursor_moved.emit(point.x(), point.y())
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MiddleButton:
            self._panning = False
            self._pan_from = None
            self.unsetCursor()
            event.accept()
            return
        super().mouseReleaseEvent(event)

    # ------------------------------------------------------------------ 차량 고정
    def set_follow(self, follow):
        """차량 고정을 켜고 끕니다. 켤 때 아는 위치가 있으면 바로 맞춥니다.

        고정 중에는 차량 헤딩이 늘 위를 향하도록 뷰 자체를 회전시킵니다(heading-up).
        끌 때는 지금 화면 가운데 있는 지점을 그대로 둔 채 회전만 북쪽 위로 되돌립니다
        -- 그래야 이후 팬 계산(mouseMoveEvent, 회전 없는 축 정렬 가정)이 맞습니다.

        위치를 이미 아는지를 돌려줍니다 -- 창이 "아직 차량 위치가 없습니다"를
        말해 줄 수 있도록.
        """
        self._follow = bool(follow)
        if self._follow and self._follow_point is not None:
            self._applied_yaw = self._follow_yaw
            self._apply_view_transform(self._current_scale(), self._follow_yaw - math.pi / 2,
                                       *self._follow_point)
        else:
            self._derotate()
        return self._follow_point is not None

    def follow_to(self, x, y, yaw):
        """차량의 새 위치/헤딩. 고정이 꺼져 있어도 기억해 둡니다(켜는 순간 쓰려고)."""
        self._follow_point = (x, y)
        self._follow_yaw = yaw
        if not self._follow:
            return
        here = self.mapFromScene(QPointF(x, y))
        center = self.viewport().rect().center()
        dyaw = math.atan2(math.sin(yaw - self._applied_yaw), math.cos(yaw - self._applied_yaw))
        if (abs(here.x() - center.x()) < self.FOLLOW_DEADZONE_PX
                and abs(here.y() - center.y()) < self.FOLLOW_DEADZONE_PX
                and abs(dyaw) < self.FOLLOW_DEADZONE_RAD):
            return
        self._applied_yaw = yaw
        self._apply_view_transform(self._current_scale(), yaw - math.pi / 2, x, y)

    def center_on_point(self, x, y):
        """씬 좌표 (x, y)를 뷰포트 한가운데로, 회전 없이(북쪽 위).

        centerOn을 쓰지 않는 이유: 그것은 sceneRect 안으로 잘리는데, sceneRect는
        itemsBoundingRect에서 자동으로 나오고 라벨/핸들이
        ItemIgnoresTransformations라 그 경계가 실제 map 범위와 다릅니다. 그래서
        뷰 변환을 직접 세우는 방식을 씁니다.
        """
        self._apply_view_transform(self._current_scale(), 0.0, x, y)

    def _derotate(self):
        """지금 화면 가운데인 씬 좌표를 그대로 둔 채 회전만 0으로(북쪽 위)."""
        self._applied_yaw = 0.0
        center = self.mapToScene(self.viewport().rect().center())
        self.center_on_point(center.x(), center.y())

    def _current_scale(self):
        """씬 1 m당 픽셀 수. 회전이 걸려 있어도 유효합니다(전단 없는 강체 변환이므로)."""
        t = self.transform()
        return math.hypot(t.m11(), t.m12())

    def _apply_view_transform(self, scale, delta, cx, cy):
        """씬 좌표 (cx, cy)가 뷰포트 한가운데에 오도록, delta(rad)만큼 회전시켜
        뷰 변환을 통째로 새로 세웁니다.

        기존 y-up 뒤집기(scale(1,-1))에 델타 회전을 얹은 것과 같습니다. 매번
        translate/rotate를 이어붙이면 누적 순서 때문에 부호가 헷갈리기 쉬워서,
        최종 아핀 계수를 직접 계산해 setTransform으로 한 번에 박습니다.
        delta == 0.0이면 지금까지의 순수 북쪽 위 변환과 정확히 같습니다.
        """
        c, s = math.cos(delta), math.sin(delta)
        m11, m12 = scale * c, scale * s
        m21, m22 = scale * s, -scale * c
        center = self.viewport().rect().center()
        dx = center.x() - m11 * cx - m21 * cy
        dy = center.y() - m12 * cx - m22 * cy
        self.setTransform(QTransform(m11, m12, m21, m22, dx, dy))

    # ------------------------------------------------------------------ 화면 맞춤
    def frame(self, rect, pad_m=5.0):
        """rect(map 프레임)가 다 보이도록 맞춥니다.

        직접 계산하는 이유: 변환이 걸린 배경 아이템은 fitInView가 쓰는 자동 경계에
        제대로 안 잡히는 경우가 있어서, 잘못 놓인 배경이 화면 밖에 있다는 힌트조차
        없이 사라집니다.
        """
        if rect is None or rect.isEmpty():
            return
        padded = rect.adjusted(-pad_m, -pad_m, pad_m, pad_m)
        self.fitInView(padded, Qt.KeepAspectRatio)
        # fitInView는 y-flip을 유지하지만 축척 한계는 안 봅니다.
        current = abs(self.transform().m11())
        if current > MAX_SCALE:
            self.scale(MAX_SCALE / current, MAX_SCALE / current)

    def meters_per_pixel(self):
        scale = self._current_scale()
        return 1.0 / scale if scale else 0.0

    def set_grid_visible(self, visible):
        self._grid_visible = bool(visible)
        self.viewport().update()

    # ------------------------------------------------------------------ 배경
    def drawBackground(self, painter, rect):
        painter.fillRect(rect, QColor(theme.COLOR_BG))
        if not self._grid_visible:
            return

        # 화면에서 대략 60 px 이상이 되는 1/2/5 계열 간격을 고릅니다.
        target_m = 60.0 * self.meters_per_pixel()
        if target_m <= 0.0:
            return
        exponent = math.floor(math.log10(target_m))
        base = 10.0 ** exponent
        for multiple in (1.0, 2.0, 5.0, 10.0):
            step = base * multiple
            if step >= target_m:
                break

        pen = QPen(QColor(theme.COLOR_GRID), 1)
        pen.setCosmetic(True)
        painter.setPen(pen)
        left = math.floor(rect.left() / step) * step
        top = math.floor(rect.top() / step) * step
        x = left
        while x <= rect.right():
            painter.drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()))
            x += step
        y = top
        while y <= rect.bottom():
            painter.drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y))
            y += step

        # 원점 축은 조금 더 밝게 -- map 프레임 기준점이 어디인지 늘 보이도록.
        axis = QPen(QColor(theme.COLOR_STALE), 1)
        axis.setCosmetic(True)
        painter.setPen(axis)
        painter.drawLine(QPointF(rect.left(), 0.0), QPointF(rect.right(), 0.0))
        painter.drawLine(QPointF(0.0, rect.top()), QPointF(0.0, rect.bottom()))
