#!/usr/bin/env python
import roslib;
import rospy
import tf
import math

from tf.msg import tfMessage
from std_msgs.msg import String 
from geometry_msgs.msg import PoseStamped

from cob_teacher.srv import *

base_to_camera_trans = [0,0,0]
base_to_camera_rot = [0,0,0,0]
calibration_succesful = False

teach_in_handle_frame_id = ""
camera_frame_id = ""

def broadcast_camera_frame(event):
	global base_to_camera_trans
	global base_to_camera_rot
	global calibration_succesful
	
	global camera_frame_id

	if (calibration_succesful == True):
		br = tf.TransformBroadcaster()

		# describing "/camera_link" wrt "/base_link"
  		br.sendTransform((base_to_camera_trans[0], base_to_camera_trans[1], base_to_camera_trans[2]),
  			(base_to_camera_rot[0], base_to_camera_rot[1], base_to_camera_rot[2], base_to_camera_rot[3]),
  			rospy.Time.now(),
  			camera_frame_id,
  			"/base_link")

def frame_calibration(req):
	global base_to_camera_trans
	global base_to_camera_rot
	global calibration_succesful

	global teach_in_handle_frame_id
	global camera_frame_id

	print "calibration requested"

	# start transform listener
	lr = tf.TransformListener()
	br = tf.TransformBroadcaster()
	
	# lookup successful?
	calibration_succesful = False

	# get frame IDs from ROS parameter server 
	teach_in_handle_frame_id = rospy.get_param('teach_in_handle_frame_id')
	camera_frame_id = rospy.get_param('camera_frame_id')

	while (calibration_succesful == False):
		current_time = rospy.Time.now()
		try:
			# look for Magelium teach_in_handle frame
			lr.waitForTransform(teach_in_handle_frame_id, camera_frame_id, current_time, rospy.Duration(2.0))
			(trans, rot) = lr.lookupTransform(teach_in_handle_frame_id, camera_frame_id, current_time)

			# describing "/camera_link" wrt "/default_handle_frame"
			i=1
			while (i< 6):
				br.sendTransform((trans[0], trans[1], trans[2]),
					(rot[0], rot[1], rot[2], rot[3]), 
				rospy.Time.now(), 
        			"/dummy_camera_link",
        			"/default_handle_frame")
				i = i + 1
				rospy.Time.sleep(0.05)

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception) as e:
			#print e
			pass

		try:
			# look for Magelium teach_in_handle frame
			lr.waitForTransform("/base_link", "/dummy_camera_link", current_time, rospy.Duration(2.0))
			(base_to_camera_trans, base_to_camera_rot) = lr.lookupTransform("/base_link", "/dummy_camera_link", current_time)
		
			calibration_succesful = True
			print "calibration successful"
			# update parameter server (scene is calibrated)
  			rospy.set_param('scene_already_calibrated', True)
  			
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception) as e:
			#print e
			pass

	return CalibrateCameraFrameResponse("success")

def calibrate_camera_frame_server():

	rospy.init_node('calibrate_camera_frame_server')
	s = rospy.Service('calibrate_camera_frame', CalibrateCameraFrame, frame_calibration)
	rospy.Timer(rospy.Duration(0.1), broadcast_camera_frame)
	print "Ready to calibrate camera frame:"

	rospy.spin()

if __name__ == "__main__":
	calibrate_camera_frame_server()
