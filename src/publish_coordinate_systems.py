#!/usr/bin/env python
import roslib;
import rospy
import tf
import math

from tf.msg import tfMessage
from std_msgs.msg import String 
from geometry_msgs.msg import PoseStamped

def publish_coordinate_systems():
    
    rospy.init_node('publish_coordinate_systems')
    
    # start transform boradcaster
    br = tf.TransformBroadcaster()

    # Pose Stamped publishing ##############################################
    #pub = rospy.Publisher('PoseBaseToCamera', PoseStamped)
    #p = PoseStamped()
    
    #quaternions = tf.transformations.quaternion_from_euler(-0.78,-0.78,0)

    #p.header.frame_id = "base_link"

    #p.pose.position.x = 0
    #p.pose.position.y = 0.395
    #p.pose.position.z = 0.49

    #p.pose.orientation.x = quaternions[0]
    #p.pose.orientation.y = quaternions[1]
    #p.pose.orientation.z = quaternions[2]
    #p.pose.orientation.w = quaternions[3]
    #######################################################################

    while not rospy.is_shutdown():
        # describing "/stereo/left" wrt "base_link"
        # axes = 'sxyz' fixed to global coord. system, axes = 'rxyz' wrt to new coord.system
        br.sendTransform((0.11, -.56, 0.825),
                    tf.transformations.quaternion_from_euler(-((45+45/4)*math.pi)/360, -((45+50)*math.pi)/360, 0, axes='rzxy'),
                    rospy.Time.now(),
                    "/stereo/left",
                    "/base_link")
        # describing "/hanle_link" wrt "base_link"
        br.sendTransform((0.40, 0.0, -0.10),
                    tf.transformations.quaternion_from_euler(0, 0, 0),
                    rospy.Time.now(),
                    "/handle_link",
                    "/base_link")
        ###################################################################
        #pub.publish(p)
        ###################################################################
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        publish_coordinate_systems()
    except rospy.ROSInterruptException:
        pass