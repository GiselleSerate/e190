#!/usr/bin/env python

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist, Vector3


def pointTracking():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    listener = tf.TransformListener()
    rospy.init_node('pointTracking', anonymous=True)

    listener.waitForTransform("/base_link", "/goal", rospy.Time(), rospy.Duration(4.0))
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            listener.waitForTransform("/base_link", "/goal", now, rospy.Duration(4.0))
            (trans,rot) = listener.lookupTransform("/base_link", "/goal", now)
            cmd = Twist()
            cmd.linear = Vector3(trans, 0, 0)
            cmd.angular = Vector3(0, 0, rot)
            pub.publish(cmd)

if __name__ == '__main__':
    try:
        robotDoLoop()
    except rospy.ROSInterruptException:
        pass
