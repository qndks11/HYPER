#!/usr/bin/env python3
"""rqt 없이 패널만 단독으로 띄웁니다: ros2 run hyper_rqt hyper_panel"""
import sys

from rqt_gui.main import Main


def main():
    return Main().main(
        sys.argv, standalone='hyper_rqt.hyper_panel.HyperPanel')


if __name__ == '__main__':
    sys.exit(main())
