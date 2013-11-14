#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_teacher')
import rospy
import tf
import math
import sys

from tf.msg import tfMessage
from std_msgs.msg import String 
from geometry_msgs.msg import PoseStamped

from cob_teacher.srv import *

def calibrate_camera_frame_client(x, y):
    rospy.wait_for_service('calibrate_camera_frame')
    try:
        calibrate_camera_frame = rospy.ServiceProxy('calibrate_camera_frame', CalibrateCameraFrame)
        result = calibrate_camera_frame(x, y)
        return result.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
	print "Usage:"
	return "%s starting_frame target_frame"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = str(sys.argv[1])
        y = str(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s to %s"%(x, y)
    print calibrate_camera_frame_client(x, y)
