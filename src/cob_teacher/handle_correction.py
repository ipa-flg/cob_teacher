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
	handle_listener = rospy.Subscriber("body_1/pose", PoseStamped, callback_handle_pose)
	handle_pub_gripper = rospy.Publisher('/corrected_handle_pose_gripper', PoseStamped)
	handle_pub_raspberry = rospy.Publisher('/corrected_handle_pose_raspberry', PoseStamped)
	listener = tf.TransformListener()
	rospy.sleep(0.1)

	rate = rospy.Rate(20.0)
	while not rospy.is_shutdown():
		corrected_handle_pose = PoseStamped()
		current_handle_pose.header.frame_id = "head_optitrack_frame"		
		# transform current pose to base_link
		if(listener.frameExists("base_link") and got_data):
			try:		
                                listener.waitForTransform("base_link", "head_optitrack_frame", rospy.Time(0), rospy.Duration(2.0))
                                current_handle_pose.header.stamp = rospy.Time(0)			
                                corrected_handle_pose = listener.transformPose("base_link", current_handle_pose)
                                # only use translational information, set roation default to (0,math.pi,0) wrt base_link
                                quat = tf.transformations.quaternion_from_euler(0, math.pi, 0)
                                corrected_handle_pose.pose.orientation.x = quat[0]
                                corrected_handle_pose.pose.orientation.y = quat[1]
                                corrected_handle_pose.pose.orientation.z = quat[2]
                                corrected_handle_pose.pose.orientation.w = quat[3]
				# do some translational corrections
                                offset_x = 0
                                offset_y = 0
                                offset_z = 0
                                corrected_handle_pose.pose.position.x = corrected_handle_pose.pose.position.x + offset_x
                                corrected_handle_pose.pose.position.y = corrected_handle_pose.pose.position.y + offset_y
                                corrected_handle_pose.pose.position.z = corrected_handle_pose.pose.position.z + offset_z
			
				#publish corrected gripper pose
                                handle_pub_gripper.publish(corrected_handle_pose)

				# publish corrected raspberry pose
                                corrected_handle_pose.pose.position.z = 1.067
                                handle_pub_raspberry.publish(corrected_handle_pose)
                	except:				
                                pass
			rate.sleep()

if __name__ == '__main__':
	try:
		correction()
	except rospy.ROSInterruptException: 
		pass
