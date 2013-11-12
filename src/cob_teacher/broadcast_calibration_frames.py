#!/usr/bin/env python
import roslib;
import rospy
import tf
import math

from tf.msg import tfMessage
from std_msgs.msg import String 
from geometry_msgs.msg import PoseStamped

########################################

# initialize handle pose in camera_link:
handle_pose_in_camera = PoseStamped()


def callback_handle_pose(pose):
    global handle_pose_in_camera
    handle_pose_in_camera = pose

def broadcast_calibration_frames():
    global handle_pose_in_camera

    rospy.init_node('broadcast_calibration_frames')
    
    # start transform boradcaster
    br = tf.TransformBroadcaster()

    # Listen:
    sub = rospy.Subscriber("MagBot/teach_in_handle_pose", PoseStamped, callback_handle_pose)
    
    while not rospy.is_shutdown():
        # describing "/handle_link" wrt "base_link"
        br.sendTransform((0.0, -0.64, 0.0),
                        tf.transformations.quaternion_from_euler(0, 0, 1.57079),
                        rospy.Time.now(),
                        "/base_link",
                        "/handle_link")
        
        #print handle_pose_in_camera
        br.sendTransform((handle_pose_in_camera.pose.position.x, handle_pose_in_camera.pose.position.y, handle_pose_in_camera.pose.position.z),
            (handle_pose_in_camera.pose.orientation.x, handle_pose_in_camera.pose.orientation.y, 
            handle_pose_in_camera.pose.orientation.z, handle_pose_in_camera.pose.orientation.w),
            rospy.Time.now(),
            "/handle_link",
            "/camera_link")
        
        rospy.sleep(0.030)
    

if __name__ == '__main__':
    try:
        broadcast_calibration_frames()
    except rospy.ROSInterruptException:
        pass