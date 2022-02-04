#!/usr/bin/env python3

import numpy as np
import tf
import rospy
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from std_msgs.msg import String, Float32, Int32, Float64, Bool

def insertData(trans, rot):
    target = Pose()
    target.position.x = trans[0]
    target.position.y = trans[1]
    target.position.z = trans[2]

    target.orientation.x = rot[0]
    target.orientation.y = rot[1]
    target.orientation.z = rot[2]
    target.orientation.w = rot[3]

    return target


if __name__=='__main__':
    rospy.init_node('bridge2sim')
    listener = tf.TransformListener()    
    rate = rospy.Rate(1.0)

    pose_est = rospy.Publisher('/pose_estimation', Pose, queue_size=1)
    pubForce=rospy.Publisher('/appliedForce',Float32, queue_size=1)

    force = Float32()
    force.data = 0

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/camera_color_optical_frame', '/handeye_target', rospy.Time(0))


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        relative_pose = insertData(trans, rot)
        pose_est.publish(relative_pose)
        pubForce.publish(force)

        rate.sleep()



