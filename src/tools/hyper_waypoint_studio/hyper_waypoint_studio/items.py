#!/usr/bin/env python3
# =====================================================================
# 씬에 올라가는 그래픽 아이템들.
#
# 씬은 map 프레임 미터이고 뷰가 scale(1, -1)로 y-up을 만듭니다. 여기서 나오는
# 규칙 두 개를 모든 아이템이 지켜야 합니다.
#
#  1) 픽셀 크기로 보여야 하는 것(글자, 핸들, 차량 표식)은 ItemIgnoresTransformations.
#     안 그러면 글자가 뒤집혀 나오고, 0.1 m짜리 핸들은 120 m 줌에서 안 보입니다.
#  2) 선은 cosmetic pen. 줌을 해도 굵기가 그대로여야 코스가 보입니다.
# =====================================================================

from python_qt_binding.QtCore import QPointF, QRectF, Qt
from python_qt_binding.QtGui import (
    QBrush, QColor, QImage, QPainterPath, QPen, QPixmap, QPolygonF, QTransform)
from python_qt_binding.QtWidgets import (
    QGraphicsItem, QGraphicsObject, QGraphicsPathItem, QGraphicsPixmapItem)

from . import theme

# z 순서. 코스는 항상 배경 위에, 차량은 항상 맨 위에.
Z_OVERLAY = 0
Z_COSTMAP = 0.5
Z_COURSE = 1
Z_LIVE_PATH = 1.5
Z_HANDLE = 2
Z_LABEL = 3
Z_VEHICLE = 4


class OverlayItem(QGraphicsPixmapItem):
    """배경 이미지. 픽셀을 다시 샘플링하지 않고 affine으로만 놓습니다.

    지오레퍼런싱은 두 가지입니다.
      extent 모드: ground.obj에서 읽은 [x0,x1,y0,y1]. x/y 축척이 따로입니다 --
                   Gazebo가 텍스처를 쿼드에 늘려 붙이므로 같은 비율로 늘려야
                   녹화한 웨이포인트가 차선 위에 얹힙니다.
      align  모드: 중심 + 가로 폭(m) + 회전. 항공사진처럼 지오레퍼런스가 아예
                   없는 이미지를 손으로 맞춘 값이고 .align.yaml에 있습니다.

    Gazebo 코스 텍스처는 이제 두 가지가 다 필요합니다 -- 쿼드가 용인 트랙에 맞춰
    69.8도 돌아가 있으면서(align) 텍스처는 여전히 쿼드 종횡비로 늘어나 있어서
    (extent) sy_ratio를 같이 넘깁니다.
    """

    def __init__(self, rgb_bytes, width_px, height_px):
        image = QImage(rgb_bytes, width_px, height_px, 3 * width_px,
                       QImage.Format_RGB888)
        # QImage가 rgb_bytes를 참조만 하므로 사본을 들고 있어야 합니다.
        self._buffer = rgb_bytes
        super().__init__(QPixmap.fromImage(image))
        self.width_px = float(width_px)
        self.height_px = float(height_px)
        self.setOffset(-self.width_px / 2.0, -self.height_px / 2.0)
        self.setZValue(Z_OVERLAY)
        self.setTransformationMode(Qt.SmoothTransformation)
        self.cx = self.cy = 0.0
        self.width_m = 1.0
        self.rot_deg = 0.0
        self._sy_ratio = 1.0     # extent 모드에서 세로/가로 축척 비

    @property
    def aspect(self):
        return self.height_px / self.width_px

    def set_extent(self, extent):
        """[x0, x1, y0, y1]로 놓습니다(회전 없음, 축척은 축마다 따로)."""
        x0, x1, y0, y1 = extent
        self.cx, self.cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        self.width_m = abs(x1 - x0)
        self.rot_deg = 0.0
        # 이미지 종횡비가 아니라 쿼드 종횡비를 따릅니다 -- 그게 Gazebo가 하는 일입니다.
        self._sy_ratio = (abs(y1 - y0) / self.width_m) / self.aspect
        self.apply()

    def set_alignment(self, cx, cy, width_m, rot_deg, sy_ratio=1.0):
        """중심/가로 폭/회전으로 놓습니다.

        sy_ratio는 세로/가로 축척 비입니다. 항공사진처럼 원본 종횡비를 지켜야 하는
        이미지는 기본값 1.0이고, 쿼드에 늘려 붙인 텍스처는 set_extent와 같은 식으로
        구한 값을 넘깁니다.
        """
        self.cx, self.cy = float(cx), float(cy)
        self.width_m = float(width_m)
        self.rot_deg = float(rot_deg)
        self._sy_ratio = float(sy_ratio)
        self.apply()

    def apply(self):
        """지금 배치를 아이템 변환으로 밀어 넣습니다.

        matplotlib의 Affine2D().scale(w).rotate_deg(r).translate(cx,cy)와 같은 배치를
        QTransform으로 쓰면 호출 순서가 반대입니다. 세로 축척이 음수인 것은 이미지가
        +y 아래인데 씬은 +y 위이기 때문입니다(matplotlib은 origin='upper'로 공짜였습니다).
        """
        sx = self.width_m / self.width_px
        sy = sx * self._sy_ratio
        self.setTransform(
            QTransform().translate(self.cx, self.cy).rotate(self.rot_deg)
            .scale(sx, -sy))

    def nudge(self, dx=0.0, dy=0.0, scale=1.0, rot=0.0):
        self.cx += dx
        self.cy += dy
        self.width_m *= scale
        self.rot_deg += rot
        self.apply()

    def map_rect(self):
        """배경이 실제로 덮는 map 프레임 사각형(회전 포함)."""
        local = QRectF(-self.width_px / 2.0, -self.height_px / 2.0,
                       self.width_px, self.height_px)
        return self.transform().mapRect(local)


class CostmapItem(QGraphicsPixmapItem):
    """nav2 local costmap 한 장. RViz의 Map 디스플레이와 같은 그림입니다.

    격자는 셀당 한 바이트이고 값이 곧 색이므로, Format_Indexed8 + 색표로 올립니다
    -- 400x400짜리를 2 Hz로 받는데 파이썬에서 픽셀을 돌면 창이 멈춥니다.

    배치는 OverlayItem과 반대입니다. OccupancyGrid의 0행은 origin 행(제일 아래)인데
    QImage의 0행은 맨 위이므로, 이 이미지는 그림으로 보면 뒤집혀 있습니다 -- 그리고
    그것이 정확히 뷰의 scale(1, -1)이 되돌리는 뒤집기입니다. 그래서 여기서는 세로
    축척이 양수입니다(사진을 놓는 OverlayItem은 음수여야 합니다). 회전이 섞일 때도
    이쪽이 맞습니다. 음수 축척은 좌우가 뒤집힌 좌표계를 돌리게 됩니다.
    """

    def __init__(self):
        super().__init__()
        self.setZValue(Z_COSTMAP)
        # 셀이 또렷해야 inflation 기울기와 lethal 띠가 구분됩니다.
        self.setTransformationMode(Qt.FastTransformation)
        self.setOpacity(0.6)
        self._table = theme.costmap_color_table()
        self.setVisible(False)

    def set_grid(self, width, height, resolution, x, y, yaw, data):
        """data는 OccupancyGrid.data 그대로(int8). -1은 255가 되어 '모름'이 됩니다."""
        import math

        import numpy as np

        if width <= 0 or height <= 0 or resolution <= 0.0:
            return False
        cells = np.frombuffer(data, dtype=np.uint8)
        if cells.size < width * height:
            return False
        cells = cells[:width * height].reshape(height, width)
        # QImage는 행이 4바이트 경계에서 시작해야 합니다. 지금 설정은 400셀이라
        # 남는 것이 없지만 resolution/width는 yaml에서 바뀔 수 있습니다.
        stride = (width + 3) & ~3
        if stride != width:
            cells = np.pad(cells, ((0, 0), (0, stride - width)),
                           constant_values=255)
        buffer = cells.tobytes()
        image = QImage(buffer, width, height, stride, QImage.Format_Indexed8)
        image.setColorTable(self._table)
        # fromImage가 복사하므로 buffer를 들고 있을 필요는 없습니다.
        self.setPixmap(QPixmap.fromImage(image))
        # offset은 기본값 (0,0) 그대로입니다: 이미지 0행 0열이 곧 격자 origin입니다.
        self.setTransform(
            QTransform().translate(x, y).rotate(math.degrees(yaw))
            .scale(resolution, resolution))
        return True


class CourseItem(QGraphicsPathItem):
    """코스 한 개의 폴리라인 + 점. 편집 중이 아닐 때 쓰는 가벼운 표현입니다.

    1500점짜리 코스 여러 개를 항상 QGraphicsItem으로 두면 못 씁니다. 편집 모드에서
    편집 중인 코스만 핸들(WaypointHandle)을 따로 만듭니다.
    """

    def __init__(self, color):
        super().__init__()
        self.setZValue(Z_COURSE)
        self._color = QColor(color)
        pen = QPen(self._color, 2)
        pen.setCosmetic(True)          # 줌해도 굵기 유지
        pen.setJoinStyle(Qt.RoundJoin)
        self.setPen(pen)
        self._dots = QGraphicsPathItem(self)
        self._dots.setZValue(Z_COURSE)
        dot_pen = QPen(self._color.lighter(130), 1)
        dot_pen.setCosmetic(True)
        self._dots.setPen(dot_pen)
        self._dots.setBrush(QBrush(Qt.NoBrush))
        self._show_dots = True

    def set_color(self, color):
        self._color = QColor(color)
        pen = self.pen()
        pen.setColor(self._color)
        self.setPen(pen)
        dot_pen = self._dots.pen()
        dot_pen.setColor(self._color.lighter(130))
        self._dots.setPen(dot_pen)

    def set_points(self, xs, ys, dot_radius_m=0.12):
        path = QPainterPath()
        if xs:
            path.moveTo(xs[0], ys[0])
            for x, y in zip(xs[1:], ys[1:]):
                path.lineTo(x, y)
        self.setPath(path)

        dots = QPainterPath()
        if self._show_dots:
            r = dot_radius_m
            for x, y in zip(xs, ys):
                dots.addEllipse(QPointF(x, y), r, r)
        self._dots.setPath(dots)

    def set_dots_visible(self, visible):
        self._show_dots = visible


class HeadingItem(QGraphicsPathItem):
    """점마다 짧은 방향 막대. 후진 구간이 눈에 보이게 하는 것이 목적입니다."""

    def __init__(self, color):
        super().__init__()
        self.setZValue(Z_COURSE)
        pen = QPen(QColor(color), 1)
        pen.setCosmetic(True)
        self.setPen(pen)

    def set_headings(self, xs, ys, yaws, length_m=0.45, stride=1):
        import math
        path = QPainterPath()
        for i in range(0, len(xs), stride):
            x, y, yaw = xs[i], ys[i], yaws[i]
            path.moveTo(x, y)
            path.lineTo(x + length_m * math.cos(yaw), y + length_m * math.sin(yaw))
        self.setPath(path)


class WaypointHandle(QGraphicsObject):
    """편집 모드에서 끌 수 있는 점 하나.

    ItemIgnoresTransformations를 켜서 화면상 크기가 줌과 무관합니다. 그래서 위치는
    아이템 좌표가 아니라 setPos로만 다루고, 그리기는 로컬 원점 주변 픽셀 단위입니다.
    """

    RADIUS_PX = 4.0
    HIT_PX = 7.0

    def __init__(self, index, x, y, color, on_moved, on_clicked):
        super().__init__()
        self.index = index
        self._color = QColor(color)
        self._on_moved = on_moved
        self._on_clicked = on_clicked
        self._reverse = False
        self.setZValue(Z_HANDLE)
        self.setPos(x, y)
        self.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setAcceptHoverEvents(True)
        self._hover = False

    def set_reverse(self, reverse):
        self._reverse = bool(reverse)
        self.update()

    def boundingRect(self):
        r = self.HIT_PX + 1.0
        return QRectF(-r, -r, 2 * r, 2 * r)

    def paint(self, painter, _option, _widget=None):
        painter.setRenderHint(painter.Antialiasing, True)
        color = QColor(theme.COLOR_BAD) if self._reverse else self._color
        if self.isSelected():
            color = QColor(theme.COLOR_LABEL_ACTIVE)
        radius = self.RADIUS_PX + (1.5 if self._hover else 0.0)
        painter.setPen(QPen(color.darker(150), 1))
        painter.setBrush(QBrush(color))
        painter.drawEllipse(QPointF(0, 0), radius, radius)

    def hoverEnterEvent(self, event):
        self._hover = True
        self.update()
        super().hoverEnterEvent(event)

    def hoverLeaveEvent(self, event):
        self._hover = False
        self.update()
        super().hoverLeaveEvent(event)

    def mousePressEvent(self, event):
        self._on_clicked(self.index)
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        super().mouseReleaseEvent(event)
        # 드래그가 끝난 자리에서 한 번만 모델에 반영합니다 -- 매 프레임 yaw를 다시
        # 구하면 실행 취소 스택이 드래그 한 번에 수십 개로 불어납니다.
        position = self.pos()
        self._on_moved(self.index, position.x(), position.y())


class LabelMarker(QGraphicsObject):
    """미션 라벨 하나. 글자가 붙으므로 변환 무시가 필수입니다."""

    SIZE_PX = 6.0

    def __init__(self, key, text, x, y, on_moved, on_clicked):
        super().__init__()
        # key는 (코스, 이름)을 담은 문자열입니다(formats.label_key). 라벨 이름은
        # 코스마다 독립이라 이름만으로는 유일하지 않습니다. text는 화면에 쓸 글자.
        self.key = key
        self.name = text
        self._on_moved = on_moved
        self._on_clicked = on_clicked
        self._active = False
        self._state = "ok"
        self.setZValue(Z_LABEL)
        self.setPos(x, y)
        self.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)

    def set_active(self, active):
        self._active = bool(active)
        self.update()

    def set_state(self, state):
        """'ok' | 'near' | 'over' -- 스냅 거리가 tolerance를 넘었는지."""
        self._state = state
        self.update()

    def _color(self):
        if self._state == "over":
            return QColor(theme.COLOR_BAD)
        if self._state == "near":
            return QColor(theme.COLOR_OK)
        return QColor(theme.COLOR_LABEL_ACTIVE if self._active else theme.COLOR_LABEL)

    def boundingRect(self):
        # 십자 표식 + 오른쪽 위 글자까지 넉넉히.
        return QRectF(-self.SIZE_PX - 2, -self.SIZE_PX - 2, 220, 2 * self.SIZE_PX + 4)

    def paint(self, painter, _option, _widget=None):
        painter.setRenderHint(painter.Antialiasing, True)
        color = self._color()
        pen = QPen(color, 2.0 if self._active else 1.5)
        painter.setPen(pen)
        s = self.SIZE_PX
        painter.drawLine(QPointF(-s, -s), QPointF(s, s))
        painter.drawLine(QPointF(-s, s), QPointF(s, -s))
        painter.setPen(QPen(color, 1))
        painter.drawText(QPointF(s + 4, s - 1), self.name)

    def mousePressEvent(self, event):
        self._on_clicked(self.key)
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        super().mouseReleaseEvent(event)
        position = self.pos()
        self._on_moved(self.key, position.x(), position.y())


class VehicleItem(QGraphicsObject):
    """차량의 현재 위치와 헤딩.

    삼각형은 화면 고정 크기(변환 무시)지만 방향은 map 프레임 yaw를 따라야 하므로,
    yaw를 뷰의 y-flip에 맞춰 화면 각도로 바꿔 직접 그립니다.
    """

    SIZE_PX = 15.0

    def __init__(self):
        super().__init__()
        self.setZValue(Z_VEHICLE)
        self.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
        self._yaw = 0.0
        self._stale = False
        self.setVisible(False)

    def set_pose(self, x, y, yaw):
        self.setPos(x, y)
        self._yaw = yaw
        self.setVisible(True)
        self.update()

    def set_stale(self, stale):
        self._stale = bool(stale)
        self.update()

    def boundingRect(self):
        r = self.SIZE_PX + 3.0
        return QRectF(-r, -r, 2 * r, 2 * r)

    def paint(self, painter, _option, _widget=None):
        import math
        painter.setRenderHint(painter.Antialiasing, True)
        color = QColor(theme.COLOR_STALE if self._stale else theme.COLOR_CAR)
        painter.setPen(QPen(color.darker(140), 1))
        painter.setBrush(QBrush(color))
        s = self.SIZE_PX
        # 아이템은 변환을 무시하므로 좌표계가 화면(y 아래로 증가)입니다.
        # map yaw를 화면 각도로 쓰려면 y를 뒤집습니다.
        a = -self._yaw
        nose = QPointF(s * math.cos(a), s * math.sin(a))
        left = QPointF(0.6 * s * math.cos(a + 2.4), 0.6 * s * math.sin(a + 2.4))
        right = QPointF(0.6 * s * math.cos(a - 2.4), 0.6 * s * math.sin(a - 2.4))
        painter.drawPolygon(QPolygonF([nose, left, right]))
