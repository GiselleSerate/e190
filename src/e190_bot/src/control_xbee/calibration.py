#!/usr/bin/env python

import rospy
import numpy as np
import math
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3

step_size = 20

def calibrate_callback(data):
    if data.data:
        pub = rospy.Publisher('/cmd_pwm', Vector3, queue_size=10)
        pwm = [-20,20]

        while pwn[1] < 256:
            pub.publish(Vector3(pwm[0], pwm[1], 0))
            rospy.sleep(5) # seconds
            pwm = [pwm[0] - step_size, pwm[1] + step_size]
            pub.publish(Vector3(0, 0, 0))
            rospy.sleep(0.5) # seconds

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('calibrate', anonymous=True)

    rospy.Subscriber("/calibrate", Bool, self.calibrate_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
