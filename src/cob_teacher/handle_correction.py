#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped

current_handle_pose = PoseStamped()
got_data = False

def callback_handle_pose(data):
	global current_handle_pose
	global got_data
	current_handle_pose = data
	got_data = True

def correction():
	global current_handle_pose
	global git_data
	rospy.init_node('handle_correction')
	handle_listener = rospy.Subscriber("/MagBot/teach_in_handle_pose", PoseStamped, callback_handle_pose)
	handle_pub = rospy.Publisher('/corrected_handle_pose', PoseStamped)
	listener = tf.TransformListener()
	rospy.sleep(0.1)

	rate = rospy.Rate(20.0)
	while not rospy.is_shutdown():
		corrected_handle_pose = PoseStamped()
		current_handle_pose.header.frame_id = "ids_camera_center"
		# transform current pose to base_link
		if(listener.frameExists("base_link") and got_data):
			current_time = rospy.Time.now()
			try:		
				listener.waitForTransform("base_link", "ids_camera_center", current_time, rospy.Duration(3.0))	
				corrected_handle_pose = listener.transformPose("base_link", current_handle_pose)
				
				# only use translational information, set roation default to (0,math.pi,0)
				quat = tf.transformations.quaternion_from_euler(0, math.pi, 0)
				corrected_handle_pose.pose.orientation.x = quat[0]
				corrected_handle_pose.pose.orientation.y = quat[1]
				corrected_handle_pose.pose.orientation.z = quat[2]
				corrected_handle_pose.pose.orientation.w = quat[3]
				# do some translational corrections
				offset_x = 0.0494
				offset_y = -0.067 + 0.018
				offset_z = 0.02
				corrected_handle_pose.pose.position.x = corrected_handle_pose.pose.position.x + offset_x
				corrected_handle_pose.pose.position.y = corrected_handle_pose.pose.position.y + offset_y
				#corrected_handle_pose.pose.position.z = corrected_handle_pose.pose.position.z + offset_z
				corrected_handle_pose.pose.position.z = 1.067				
				#publish corrected pose
				handle_pub.publish(corrected_handle_pose)

			except:
				pass
		rate.sleep()

if __name__ == '__main__':
	try:
		correction()
	except rospy.ROSInterruptException: 
		pass
