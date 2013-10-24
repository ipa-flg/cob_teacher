import yaml
import yaml.constructor
import roslib
import rosmsg

from FieldUpdater import *

supported_types = ['geometry_msgs/PoseStamped', 'std_msgs/String', 'std_msgs/Float64', 'trajectory_msgs/JointTrajectory']
supportedUpdaters = [PoseStampedUpdater, StringUpdater, FloatUpdater, StdStringUpdater] 

class YamlManager():
	def __init__(self, filename):
		self.filename = filename
		self.fields = []
		self.field_data = []
		stream = file(self.filename, 'r') 
		self.data = yaml.load(stream)
		for d in self.data:
			self.fields.append(d)
			self.field_data.append(self.data[d])
		stream.close()
		#print self.fields
		#print self.field_data
		#print self.data

	def getFields(self):
		return self.fields

	def getData(self):
		return self.data

	def getTypes(self):
		msgtypes = []
		for dat in self.fields:
			msgtypes.append(self.getType(dat))
		return msgtypes

	def getType(self, fieldname):
		dat = self.data[fieldname]
		if type(dat) == dict:
			return self.compareType(self.field_data[0])
		elif type(dat) == str:
			return "string"
		elif type(dat) == float:
			return "float"
		else:
			return type(dat)

	def compareType(self, dat):
		for t in supported_types:
			msg_class = roslib.message.get_message_class(t)
			instance = msg_class()
			typestring = yaml.load(rosmsg.get_yaml_for_msg(instance, flow_style_=1))
			if(len(set(typestring) ^ set(dat)) == 0):
				return t
		return "Unknown"

	def updateField(self, fieldname, data):		
		current_fieldtype = self.getType(fieldname)
		for updater in supportedUpdaters:
			if(updater().getType() == current_fieldtype):
				self.data = updater().update(fieldname, self.data, data)


	def writeFile(self):
		with open(self.filename, "w") as stream:
			stream.write(yaml.dump(self.data, default_flow_style=False))
			stream.close()

