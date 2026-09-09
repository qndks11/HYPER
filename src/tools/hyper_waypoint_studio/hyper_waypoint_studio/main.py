#!/usr/bin/env python3
# =====================================================================
# 진입점.
#
# ROS는 선택입니다. 보기/편집이 이 프로그램의 기본 용도이므로, rclpy를 못 올려도
# (또는 --no-ros로 껐어도) 창은 그대로 뜨고 파일 기능은 전부 돕니다. 녹화/주행
# 모드만 꺼집니다.
#
# 신호 처리가 두 갈래인 이유:
#   SIGTERM -> SIG_DFL. launch가 스택을 내릴 때 이 창만 안 죽고 남으면 다음 실행에서
#             레코더 서비스에 붙은 유령 GUI가 됩니다. 저장 여부를 묻는 모달이
#             launch 종료를 막아서는 안 되므로 즉시 죽습니다.
#   SIGINT  -> 창 닫기. 터미널에서 Ctrl-C를 눌렀을 때는 저장 안 한 편집을 물어볼
#             기회를 줍니다. 파이썬 핸들러는 Qt 이벤트 루프가 도는 동안 실행되지
#             않으므로, 아무것도 안 하는 타이머로 인터프리터에 틈을 만들어 줍니다.
# =====================================================================

import argparse
import os
import signal
import sys


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        description='HYPER waypoint studio -- 코스 보기/편집/녹화/주행')
    parser.add_argument('courses', nargs='*',
                        help='열어 둘 웨이포인트 CSV (여러 개 가능)')
    parser.add_argument('--mission', help='열어 둘 mission.yaml')
    parser.add_argument('--overlay',
                        help="배경 이미지 경로, 또는 'gazebo'(시뮬 코스 텍스처)")
    parser.add_argument('--mode', default='view', choices=('view', 'edit', 'record', 'drive'),
                        help='시작 모드 (기본: view)')
    parser.add_argument('--destination', default='',
                        help='녹화 모드의 저장 파일 초기값')
    parser.add_argument('--recorder', default='/waypoint_recorder')
    parser.add_argument('--mission-manager', default='/mission_manager')
    parser.add_argument('--teleport', default='/teleport_service')
    parser.add_argument('--pose-topic', default='/odometry/filtered_map')
    parser.add_argument('--no-ros', action='store_true',
                        help='rclpy를 아예 올리지 않습니다(순수 보기/편집)')
    # launch가 붙이는 --ros-args 뭉치는 여기서 볼 일이 없습니다.
    known, _ = parser.parse_known_args(
        [a for a in argv[1:] if a != '--ros-args'])
    return known


def main(argv=None):
    argv = list(argv if argv is not None else sys.argv)
    args = _parse_args(argv)

    from python_qt_binding.QtCore import QTimer
    from python_qt_binding.QtWidgets import QApplication

    from .app_window import StudioWindow
    from . import ros_link

    app = QApplication(argv)
    app.setApplicationName('waypoint studio')

    if args.no_ros:
        link = ros_link.NullRosLink('--no-ros로 실행 중입니다')
    else:
        link = ros_link.create(
            recorder=args.recorder, manager=args.mission_manager,
            teleport=args.teleport, pose_topic=args.pose_topic)

    window = StudioWindow(
        link, initial_mode=args.mode, courses=args.courses, mission=args.mission,
        overlay=args.overlay, destination=args.destination)
    window.show()

    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    signal.signal(signal.SIGINT, lambda *_: app.closeAllWindows())
    # 파이썬 핸들러가 돌 틈. 이게 없으면 Qt 루프가 도는 동안 Ctrl-C가 먹지 않습니다.
    nudge = QTimer()
    nudge.timeout.connect(lambda: None)
    nudge.start(200)

    code = app.exec_()
    try:
        import rclpy
        if rclpy.ok():
            rclpy.try_shutdown()
    except Exception:                          # noqa: BLE001
        pass
    return code


if __name__ == '__main__':
    sys.exit(main())
