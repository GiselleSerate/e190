#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Point, Quaternion


class goal_pose_transformer:

    def __init__(self):
        # Init global goal
        self.goalPoseGlobal = PoseStamped()
        self.goalPoseGlobal.header.frame_id = "/odom"
        self.goalPoseGlobal.pose.position = Point(2.0, 2.0, 0.0)
        self.goalPoseGlobal.pose.orientation = Quaternion(0.0, 0.0, 1.0, 0.0)
        
        # Init local goal
        self.goalPoseLocal = PoseStamped()
        self.goalPoseLocal.header.frame_id = "/base_link"

        rospy.init_node('goal_pose_transform', anonymous=True)
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.listener.waitForTransform("/base_link", "/odom", rospy.Time(0), rospy.Duration(2.0))
            # Now that we have the transform, transform the global goal into base_link
            self.goalPoseLocal = self.listener.transformPose("base_link",self.goalPoseGlobal)
            print("Position:\n"+str(self.goalPoseLocal.pose.position)+"\nOrientation:\n"+str(self.goalPoseLocal.pose.orientation))
            self.rate.sleep();



if __name__ == '__main__':
    try:
        goal_pose_transformer()

    except (rospy.ROSInterruptException):
        pass