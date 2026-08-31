#!/usr/bin/env python3
"""rqt 플러그인 진입점. 실제 내용은 panel_widget.py에 있습니다."""
import os

from ament_index_python.packages import get_package_share_directory
from python_qt_binding.QtWidgets import QFileDialog, QMessageBox
from rqt_gui_py.plugin import Plugin

from hyper_rqt.panel_widget import HyperPanelWidget

# 설정 파일을 찾는 순서. 첫 번째로 존재하는 것을 씁니다.
#   1) --config <경로>            (rqt 실행 인자)
#   2) 톱니바퀴로 고른 경로       (rqt settings에 저장됨)
#   3) $HYPER_RQT_CONFIG
#   4) share/hyper_rqt/config/panel.yaml
_SETTINGS_KEY = 'config_path'


def _default_config():
    return os.path.join(
        get_package_share_directory('hyper_rqt'), 'config', 'panel.yaml')


def _config_from_argv(argv):
    for index, arg in enumerate(argv):
        if arg == '--config' and index + 1 < len(argv):
            return argv[index + 1]
        if arg.startswith('--config='):
            return arg.split('=', 1)[1]
    return None


class HyperPanel(Plugin):

    def __init__(self, context):
        super(HyperPanel, self).__init__(context)
        self.setObjectName('HyperPanel')

        assert hasattr(context, 'node'), 'Context does not have a node.'
        self._context = context
        self._widget = None
        self._config_path = (
            _config_from_argv(context.argv())
            or os.environ.get('HYPER_RQT_CONFIG')
            or _default_config())
        self._create_widget()

    def _create_widget(self):
        try:
            widget = HyperPanelWidget(self._context.node, self._config_path)
        except Exception as exc:  # noqa: BLE001 -- YAML 오타로 rqt 전체가 죽지 않게
            QMessageBox.critical(
                None, 'HYPER Panel',
                '설정을 읽지 못했습니다:\n{}\n\n{}'.format(self._config_path, exc))
            raise

        if self._context.serial_number() > 1:
            widget.setWindowTitle(
                widget.windowTitle() + ' ({})'.format(self._context.serial_number()))
        if self._widget is not None:
            self._context.remove_widget(self._widget)
            self._widget.shutdown()
        self._widget = widget
        self._context.add_widget(widget)

    def trigger_configuration(self):
        path, _filter = QFileDialog.getOpenFileName(
            self._widget, 'panel.yaml 고르기',
            os.path.dirname(self._config_path), 'YAML (*.yaml *.yml)')
        if not path:
            return
        previous = self._config_path
        self._config_path = path
        try:
            self._create_widget()
        except Exception:  # noqa: BLE001 -- 실패하면 이전 설정으로 되돌립니다
            self._config_path = previous
            self._create_widget()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value(_SETTINGS_KEY, self._config_path)

    def restore_settings(self, plugin_settings, instance_settings):
        saved = instance_settings.value(_SETTINGS_KEY, None)
        # --config가 있으면 그쪽이 이깁니다 -- 명시적으로 준 인자를 저장된 값이
        # 덮으면 왜 안 바뀌는지 알 수 없게 됩니다.
        if saved and not _config_from_argv(self._context.argv()):
            if saved != self._config_path and os.path.exists(saved):
                self._config_path = saved
                self._create_widget()

    def shutdown_plugin(self):
        if self._widget is not None:
            self._widget.shutdown()
