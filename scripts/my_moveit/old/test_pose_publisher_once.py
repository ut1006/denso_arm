#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose


def talker():
    rospy.init_node("gibbe_test_pose_once_publisher")
    pub = rospy.Publisher('/gibbe/arm/goal', Pose, queue_size=1)
    message = Pose()
    message.position.x = 0.5
    message.position.y = 0.2
    message.position.z = 0.4
    pub.publish(message)
    rospy.loginfo("test_pose " + str(message.position.x))

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass