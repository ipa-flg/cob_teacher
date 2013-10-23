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

    plugin_chooser = []

    def __init__(self, context):
        super(cob_teacher_plugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('cob_teacher_plugin')
        self._widget = QWidget()
        self._widget.setObjectName('cob_teacher_plugin')
        grid = QtGui.QGridLayout()
        self.group_layout = QtGui.QVBoxLayout()
        grid.setSpacing(4)


        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
        args = self._parse_args(context.argv())
        for config_file in args.config_file:
            print config_file
            self.ym = YamlManager(config_file)
            for field in self.ym.getFields():
                self.group_layout.addWidget(self.getFieldWidget(field))
        self._widget.setLayout(self.group_layout)

    def getFieldWidget(self, field):
        group = QtGui.QGroupBox()
        group.setTitle(field+":")
        field_layout = QtGui.QVBoxLayout()
        group.setLayout(field_layout)
        teachers_found = self.findTeachers(field)
        if(len(teachers_found) > 1):
            print "Several plugins were found for fieldtype " + self.ym.getType(field)
            combo = QtGui.QComboBox()
            combo.addItem("")
            for teacher in teachers_found:
                combo.addItem(teacher().getName())
            field_layout.addWidget(combo)
            self.plugin_chooser.append({"name": field, "layout": field_layout, "widget": None, "chooser": combo})
            self.connect(combo, QtCore.SIGNAL('activated(QString)'), self.combo_chosen)
        else:
            teach_widget = teachers_found[0]().getRQTWidget(field, self.ym.data[field])
            field_layout.addWidget(teach_widget)
        return group
            


    def combo_chosen(self, text):
        sender = self.sender()
        for chooser in self.plugin_chooser:
            if(chooser["chooser"] == sender):
                print "Chosen: ", chooser["name"]
                for teacher in availableTeachers:
                    if(teacher().getName() == text):
                        print "Chosen: ", teacher().getName()
                        teach_widget = teacher().getRQTWidget(chooser["name"], self.ym.data[chooser["name"]])
                        if(teach_widget != None):
                            #remove currently activated plugin
                            if(chooser["widget"] != None):
                                chooser["widget"].setParent(None)
                            chooser["widget"] = teach_widget
                            chooser["layout"].addWidget(teach_widget)
    

    def findTeachers(self, fieldName):
        teachers_found = []
        for teacher in availableTeachers:
            if(teacher().getType() == self.ym.getType(fieldName)):
                teachers_found.append(teacher)        
        return teachers_found

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