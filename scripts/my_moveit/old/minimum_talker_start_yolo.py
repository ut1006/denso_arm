import rospy
import std_msgs.msg

rospy.init_node("talker")
pub = rospy.Publisher("/recognition_arm/start_cluster_processing", std_msgs.msg.Bool, queue_size=1)
rate = rospy.Rate(0.5)
while not rospy.is_shutdown():
    pub.publish(True)
    print("hi")
    rate.sleep()