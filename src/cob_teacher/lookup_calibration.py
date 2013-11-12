#!/usr/bin/env python
import roslib;
import rospy
import tf
import math

from tf.msg import tfMessage
from std_msgs.msg import String 
from geometry_msgs.msg import PoseStamped
#############################################


def lookup_calibration():
    global handle_pose_in_camera

    rospy.init_node('lookup_calibration')
    
    # start transform listener
    lr = tf.TransformListener()
    
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        try:
            lr.waitForTransform("/base_link", "/camera_link", current_time, rospy.Duration(2.0))
            (trans, rot) = lr.lookupTransform("/base_link", "/camera_link", current_time)
            print "Transformation: "
            print trans
            print "Rotation: " 
            print rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception) as e:
            print e
            pass

        ###################################################################
        rospy.sleep(0.030)
    

if __name__ == '__main__':
    try:
        lookup_calibration()
    except rospy.ROSInterruptException:
        pass