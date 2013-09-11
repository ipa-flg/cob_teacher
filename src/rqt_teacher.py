import os
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

class cob_teacher_plugin(Plugin):

    def __init__(self, context):
        super(cob_teacher_plugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('cob_teacher_plugin')
        self._widget = QWidget()
        self._widget.setObjectName('cob_teacher_plugin')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass