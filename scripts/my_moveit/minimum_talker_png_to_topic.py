#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def send_image():
    # img = cv2.imread("./image/0070_real.PNG")
    img = cv2.imread("/home/kamiya/Documents/YOLOv5/yolov5/data/images/matsumoto_1631837221.0.png")
    bridge = CvBridge()
    image_message = bridge.cv2_to_imgmsg(img, "bgr8")
    pub = rospy.Publisher('/zem/zed_node/left/image_rect_color', Image, queue_size=1)
    rospy.init_node("dummy_image_publisher")
    while not rospy.is_shutdown():
        pub.publish(image_message)
        rospy.loginfo("image published")
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        send_image()
    except rospy.ROSInterruptException: pass