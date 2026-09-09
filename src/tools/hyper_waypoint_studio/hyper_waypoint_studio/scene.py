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
from python_qt_binding.QtGui import QColor, QPainter, QPen
from python_qt_binding.QtWidgets import QGraphicsScene, QGraphicsView

from . import theme

# 한 번에 볼 수 있는 범위. 너무 넓게 줌아웃하면 아무것도 안 보이고, 너무 좁으면
# 핸들 하나가 화면을 채웁니다.
MIN_SCALE = 0.05      # 픽셀당 20 m
MAX_SCALE = 200.0     # 픽셀당 0.5 cm


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

    def __init__(self, scene):
        super().__init__(scene)
        self.setRenderHint(QPainter.Antialiasing, True)
        self.setDragMode(QGraphicsView.RubberBandDrag)
        self.setTransformationAnchor(QGraphicsView.NoAnchor)
        self.setResizeAnchor(QGraphicsView.NoAnchor)
        self.setViewportUpdateMode(QGraphicsView.SmartViewportUpdate)
        self.setMouseTracking(True)
        self.setFrameStyle(0)
        # y-up. 이 한 줄이 이 파일의 거의 모든 주의사항의 원인입니다.
        self.scale(1.0, -1.0)
        self._panning = False
        self._pan_from = None
        self._grid_visible = True

    # ------------------------------------------------------------------ 줌/팬
    def wheelEvent(self, event):
        factor = 1.2 if event.angleDelta().y() > 0 else 1.0 / 1.2
        current = self.transform().m11()
        if not (MIN_SCALE <= current * factor <= MAX_SCALE):
            return
        # 커서 아래 지점이 제자리에 있도록 줌합니다.
        anchor = self.mapToScene(event.pos())
        self.scale(factor, factor)
        moved = self.mapToScene(event.pos())
        self.translate(moved.x() - anchor.x(), moved.y() - anchor.y())

    def mousePressEvent(self, event):
        if event.button() == Qt.MiddleButton:
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
        scale = abs(self.transform().m11())
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
