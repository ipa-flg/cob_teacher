#!/usr/bin/python

import sys
import roslib
roslib.load_manifest('cob_script_server')

from geometry_msgs.msg import PoseStamped
from teachers import *
from YamlManager import *

availableTeachers = [StringTeacher, PoseInputTeacher, PoseTouchupTeacher]

class SimpleTeacher:
	def __init__(self, filename):
		ym = YamlManager(filename)
		for field in ym.getFields():
			print "Teaching", field, ":"
			teacher = self.findTeacher(ym.getType(field))
			if(teacher != None):
				teacher().getData(field)


	def findTeacher(self, fieldtype):
		for teacher in availableTeachers:
			if(teacher().getType() == fieldtype):
				print "FoundPlugin"
				return teacher

if __name__ == '__main__':
	if(len(sys.argv) != 2):
		print "usage: cob_teacher.py yaml_filename"
	else:
		st = SimpleTeacher(sys.argv[1])
		