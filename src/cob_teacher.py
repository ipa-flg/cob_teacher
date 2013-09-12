#!/usr/bin/python

import sys
import roslib
roslib.load_manifest('cob_script_server')

from geometry_msgs.msg import PoseStamped
from teachers import *
from YamlManager import *

availableTeachers = [StringInputTeacher, FloatInputTeacher, PoseInputTeacher, PoseTouchupTeacher]

class SimpleTeacher:
	def __init__(self, filename):
		ym = YamlManager(filename)
		for field in ym.getFields():
			print " "
			print "Teaching", field, "(" + ym.getType(field) + "):"
			teacher = self.findTeacher(ym.getType(field))
			if(teacher != None):
				p = teacher().getData(field)
				print p

	def findTeacher(self, fieldtype):
		self.teachers = []
		for teacher in availableTeachers:
			if(teacher().getType() == fieldtype):
				self.teachers.append(teacher)
		# check whether several plugins were found with common fieldtype 
		if(len(self.teachers) > 1):
			print "Several plugins were found for fieldtype " + fieldtype
			return self.selectTeacher(self.teachers)
		else:
			if len(self.teachers) == 1:
				print "Found plugin"
				return self.teachers[0]

	# in case several plugins were found, select teacher by input
	def selectTeacher(self,teachers):
		print "Please select TeacherPlugin: "
		print " "
		k = 0
		for teacher in teachers:
			print "	for " + teacher().getName() + " type " + str(k)
			k = k + 1
		print " "
		k = int(input("Input: "))
		if(k >= 0 & k < len(teachers)):
			return teachers[k]
		else:
			return None


if __name__ == '__main__':
	if(len(sys.argv) != 2):
		print "usage: cob_teacher.py yaml_filename"
	else:
		st = SimpleTeacher(sys.argv[1])
		
