#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped

current_handle_pose = PoseStamped()

def callback_handle_pose(data):
    global current_handle_pose
    current_handle_pose = data

def correction():
	global current_handle_pose
	rospy.init_node('handle_correction')
	handle_listener = rospy.Subscriber("/MagBot/teach_in_handle_pose", PoseStamped, callback_handle_pose)
	handle_pub = rospy.Publisher('/corrected_handle_pose', PoseStamped)
	listener = tf.TransformListener()
	rospy.sleep(0.1)

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		corrected_handle_pose = PoseStamped()

		# transform current pose to base_link
		corrected_handle_pose = listener.transformPose("base_link", current_handle_pose)

		# only use translational information, set roation default to (0,math.pi,0)
		quat = tf.transformations.quaternion_from_euler(0, math.pi, 0)
		corrected_handle_pose.pose.orientation.x = quat[0]
		corrected_handle_pose.pose.orientation.y = quat[1]
		corrected_handle_pose.pose.orientation.z = quat[2]
		corrected_handle_pose.pose.orientation.w = quat[3]

		# do some translational corrections
		# ...

		#publish corrected pose
		handle_pub.publish(corrected_handle_pose)

if __name__ == '__main__':
    try:
    	correction()
    except rospy.ROSInterruptException: pass