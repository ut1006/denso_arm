import rospy
import std_msgs.msg

rospy.init_node("talker")
pub = rospy.Publisher("chat", std_msgs.msg.String, queue_size=1)
rate = rospy.Rate(2)
while not rospy.is_shutdown():
    pub.publish("hello")
    rate.sleep()