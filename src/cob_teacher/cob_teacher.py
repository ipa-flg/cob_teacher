#!/usr/bin/python

import sys
import roslib
roslib.load_manifest('cob_script_server')

from geometry_msgs.msg import PoseStamped
from teachers import *
from YamlManager import *

availableTeachers = [StringInputTeacher, FloatInputTeacher, IntInputTeacher, PoseInputTeacher, 
					 PoseTouchupTeacher, PoseTeachInHandleTeacher]

class SimpleTeacher:
	def __init__(self, filename):
		
		# init cob_teacher node
		rospy.init_node("cob_teacher", anonymous=True)

		ym = YamlManager(filename)
		for field in ym.getFields():
			print " "
			print "Teaching", field, "(" + ym.getType(field) + "):"
			teacher = self.findTeacher(ym.getType(field))
			if(teacher != None):
				p = teacher().getData(field)
				ym.updateField(field, p)
		ym.writeFile()

	def findTeacher(self, fieldtype):
		teachers_found = []
		for teacher in availableTeachers:
			if(teacher().getType() == fieldtype):
				teachers_found.append(teacher)
		# check whether several plugins were found with common fieldtype 
		if(len(teachers_found) > 1):
			print "Several plugins were found for fieldtype " + fieldtype
			return self.selectTeacherbyInput(teachers_found)
		else:
			if len(teachers_found) == 1:
				print "Found plugin"
				return teachers_found[0]

	# in case several plugins were found, select teacher by input
	def selectTeacherbyInput(self, teachers):
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
		
