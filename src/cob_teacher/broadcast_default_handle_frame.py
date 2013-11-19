#!/usr/bin/env python
import roslib;
import rospy
import tf
import math

from tf.msg import tfMessage
from std_msgs.msg import String 
from geometry_msgs.msg import PoseStamped
#############################################

def broadcast_default_handle_frame():

    # init ROS node
    rospy.init_node('broadcast_default_handle_frame')
    
    # start transform boradcaster
    br = tf.TransformBroadcaster()

    ##############################################################
    rospy.set_param('camera_frame_id', '/stereo/left')
    rospy.set_param('teach_in_handle_frame_id', '/teach_in_handle')
    rospy.set_param('IPA_teach_in_handle_frame_id', '/IPA_teach_in_handle')
    rospy.set_param('scene_already_calibrated', False)
    ##############################################################

    while not rospy.is_shutdown():
        # describing "/default_handle_link" wrt "/base_link"
        br.sendTransform((0.64, 0.0, 0.0),
                        tf.transformations.quaternion_from_euler(0, 0, -1.57079),
                        rospy.Time.now(),
                        "/default_handle_frame",
                        "/base_link")

        # broadcasting IPA_handle_frame 
        br.sendTransform((0, 0, 0),
                        tf.transformations.quaternion_from_euler(0, 3.14, 0),
                        rospy.Time.now(),
                        "/IPA_teach_in_handle",
                        "/teach_in_handle")

        rospy.sleep(0.030)
    
if __name__ == '__main__':
    try:
        broadcast_default_handle_frame()
    except rospy.ROSInterruptException:
        pass