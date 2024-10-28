#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool


class PosePublisher:
    def __init__(self):
        self.quit_publish = False   
        self.pub = rospy.Publisher("/gibbe/arm/goal", Pose, queue_size=1)
        rospy.init_node("gibbe_test_pose_publisher")

        
    def talker(self):
        message = Pose()
        message.position.x = 0.5
        message.position.y = 0.2
        message.position.z = 0.6
        while not rospy.is_shutdown() and not self.quit_publish:
            self.pub.publish(message)
            rospy.loginfo("test_pose " + str(message.position.x))
            rospy.sleep(1.0)


    def callback(self, data):
        self.quit_publish = data
    
    def subscribe(self):
        print("subscribe")
        rospy.Subscriber("/gibbe/arm/quit/goal", Bool, self.callback)

if __name__ == "__main__":
    publisher = PosePublisher()
    publisher.subscribe()
    publisher.talker()
    rospy.spin()
