#!/usr/bin/env python3
# 실차에서 gps_accuracy_gui와 나란히 띄우므로 waypoint_record_gui.py와 같은 팔레트를 씁니다.

COLOR_TEXT = '#c9d1d9'
COLOR_BG = '#0d1117'
COLOR_PANEL = '#161b22'
COLOR_GOOD = '#3fb950'
COLOR_OK = '#d29922'
COLOR_BAD = '#f85149'
COLOR_STALE = '#6e7681'
COLOR_PATH = '#58a6ff'
COLOR_CAR = '#bc8cff'
COLOR_PREV = '#8b949e'
COLOR_LABEL = '#f0883e'
COLOR_LABEL_ACTIVE = '#f85149'
COLOR_GRID = '#21262d'

# 코스 레이어에 순서대로 배정하는 색. 배경(아스팔트/잔디) 위에서도 서로 구분되고
# 라벨(주황)/차량(보라)과 겹치지 않는 것들로 골랐습니다.
COURSE_COLORS = (
    '#00e5ff', '#3fb950', '#e3b341', '#ff7b72',
    '#a5d6ff', '#d2a8ff', '#7ee787', '#ffa657',
)

# 코스트맵 색. RViz2 map display의 "costmap" 배색을 그대로 옮겼습니다 -- 스튜디오와
# follow_path.rviz를 나란히 놓고 봐도 같은 그림이어야 하기 때문입니다.
#
# 값의 의미(nav2가 0-255를 0-100으로 줄여 보냅니다):
#   0        비어 있음        -> 투명 (배경 이미지가 비쳐야 합니다)
#   1-98     inflation 기울기 -> 파랑 -> 빨강
#   99       inscribed(253)   -> 하늘
#   100      lethal(254)      -> 자홍
#   -1(255)  모름             -> 투명
# drivable_area_layer가 쓰는 200은 여기서 78-79로 들어옵니다(주황빛 빨강). 라이다
# lethal(자홍)과 색이 달라서 차선 밖 영역인지 실제 장애물인지 구분이 됩니다.


def costmap_color_table():
    """QImage.setColorTable에 넣을 256칸 0xAARRGGBB 목록."""
    table = [0x00000000] * 256
    for i in range(1, 99):
        v = (255 * i) // 100
        table[i] = 0xFF000000 | (v << 16) | (255 - v)
    table[99] = 0xFF00FFFF
    table[100] = 0xFFFF00FF
    for i in range(101, 128):
        table[i] = 0xFFFF0000
    for i in range(128, 255):
        table[i] = 0xFF704D00      # 나올 일이 없는 값. 나오면 눈에 띄라고 둡니다.
    return table


# /gps/fix의 NavSatStatus. 녹화 품질은 결국 이 값입니다.
GPS_STATUS = {
    -1: ('NO FIX', COLOR_BAD),
    0: ('단독측위', COLOR_OK),
    1: ('SBAS', COLOR_OK),
    2: ('RTK / GBAS', COLOR_GOOD),
}

STALE_S = 2.0

WINDOW_STYLE = f"""
QMainWindow, QWidget {{ background-color: {COLOR_BG}; color: {COLOR_TEXT}; }}
QDockWidget {{ color: {COLOR_TEXT}; titlebar-close-icon: none; }}
QDockWidget::title {{ background: {COLOR_PANEL}; padding: 5px; }}
QGroupBox {{ border: 1px solid {COLOR_GRID}; border-radius: 4px; margin-top: 8px;
             padding-top: 8px; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 8px; color: {COLOR_STALE}; }}
QPushButton {{ background-color: {COLOR_PANEL}; border: 1px solid {COLOR_GRID};
               border-radius: 4px; padding: 5px 10px; }}
QPushButton:hover:enabled {{ border-color: {COLOR_STALE}; }}
QPushButton:disabled {{ color: {COLOR_STALE}; }}
QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QListWidget, QPlainTextEdit {{
    background-color: {COLOR_PANEL}; color: {COLOR_TEXT};
    border: 1px solid {COLOR_GRID}; border-radius: 4px; padding: 3px; }}
QListWidget::item:selected {{ background-color: #1f6feb; color: white; }}
QToolBar {{ background: {COLOR_PANEL}; border: none; spacing: 4px; padding: 3px; }}
QToolButton {{ padding: 5px 12px; border-radius: 4px; }}
QToolButton:checked {{ background-color: #1f6feb; color: white; }}
QStatusBar {{ background: {COLOR_PANEL}; color: {COLOR_STALE}; }}
QMenuBar {{ background: {COLOR_PANEL}; }}
QMenuBar::item:selected {{ background: #1f6feb; }}
QMenu {{ background: {COLOR_PANEL}; border: 1px solid {COLOR_GRID}; }}
QMenu::item:selected {{ background: #1f6feb; }}
QHeaderView::section {{ background: {COLOR_PANEL}; color: {COLOR_STALE};
                        border: none; padding: 3px; }}
QTableWidget {{ background: {COLOR_PANEL}; gridline-color: {COLOR_GRID};
                border: 1px solid {COLOR_GRID}; }}
"""
