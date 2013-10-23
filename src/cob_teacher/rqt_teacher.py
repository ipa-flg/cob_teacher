import os
import rospy
import argparse

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from PyQt4 import QtGui, QtCore

from teachers import *
from YamlManager import *


availableTeachers = [StringInputTeacher, FloatInputTeacher, PoseInputTeacher, 
                     PoseTouchupTeacher, PoseTeachInHandleTeacher]

class cob_teacher_plugin(Plugin):

    def __init__(self, context):
        super(cob_teacher_plugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('cob_teacher_plugin')
        self._widget = QWidget()
        self._widget.setObjectName('cob_teacher_plugin')
        grid = QtGui.QGridLayout()
        group_layout = QtGui.QVBoxLayout()
        grid.setSpacing(4)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
        args = self._parse_args(context.argv())
        for config_file in args.config_file:
            print config_file
            ym = YamlManager(config_file)
            for field in ym.getFields():
                teacher = self.findTeacher(ym.getType(field))
                if(teacher != None):
                    #p = teacher().getData(field)
                    teach_widget = teacher().getRQTWidget(field, ym.data[field])
                    #field_edit = QtGui.QLineEdit(str(ym.data[field]))
                    group_layout.addWidget(teach_widget)
                    #grid.addWidget(field_edit, count, 1)
        self._widget.setLayout(group_layout)


    def findTeacher(self, fieldtype):
        teachers_found = []
        for teacher in availableTeachers:
            if(teacher().getType() == fieldtype):
                teachers_found.append(teacher)
        # check whether several plugins were found with common fieldtype 
        if(len(teachers_found) > 1):
            print "Several plugins were found for fieldtype " + fieldtype
            return teachers_found[0]
            #return self.selectTeacherbyInput(teachers_found)
        else:
            if len(teachers_found) == 1:
                print "Found plugin"
                return teachers_found[0]


    def _parse_args(self, argv):
        parser = argparse.ArgumentParser(prog='cob_teacher', add_help=False)
        cob_teacher_plugin.add_arguments(parser)
        return parser.parse_args(argv)

    @staticmethod
    def add_arguments(parser):
        group = parser.add_argument_group('Options for cob_teacher plugin')
        group.add_argument('config_file', nargs='*', default=[], help='Configfile to load')

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