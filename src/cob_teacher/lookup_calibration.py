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

    rospy.init_node('lookup_calibration')
    
    # start transform listener
    lr = tf.TransformListener()
    br = tf.TransformBroadcaster()
    
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        try:
            # look for Magelium teach_in_handle frame
            lr.waitForTransform("/teach_in_handle", "/stereo/left", current_time, rospy.Duration(2.0))
            (trans, rot) = lr.lookupTransform("/teach_in_handle", "/stereo/left", current_time)

            # describing "/camera_link" wrt "/default_handle_frame"
            br.sendTransform((trans[0], trans[1], trans[2]), (rot[0], rot[1], rot[2], rot[3]),
                rospy.Time.now(), 
                "/camera_link", 
                "/default_handle_frame")

           

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception) as e:
            print e
            pass


        try:
             # look for Magelium teach_in_handle frame
            lr.waitForTransform("/base_link", "/camera_link", current_time, rospy.Duration(2.0))
            (trans, rot) = lr.lookupTransform("/base_link", "/camera_link", current_time)

            print trans

            # describing "/camera_link" wrt "/base_link"
            #br.sendTransform((trans[0], trans[1], trans[2]), (rot[0], rot[1], rot[2], rot[3]),
            #    rospy.Time.now(),  
            #    "/base_link",
            #    "/camera_link")

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