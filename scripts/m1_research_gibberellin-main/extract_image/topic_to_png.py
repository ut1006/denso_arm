#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import math

class SaveImage:
    def __init__(self):
        self.new_image = [0, 0]
        self.right_image = Image()
        self.left_image = Image()

    def save_image(self):
        print(self.new_image)
        if(self.new_image[0] == 1 & self.new_image[1] == 1):
            try:
                self.new_image = [2, 2]
                print('image saved')
                bridge = CvBridge()
                left_img = bridge.imgmsg_to_cv2(self.left_image, "bgr8")
                right_img = bridge.imgmsg_to_cv2(self.right_image, "bgr8")
                self.new_image = [0, 0]
                # img = cv2.cvtColor(raw, cv2.COLOR_BGR2RGB)
                timestamp = time.time()
                # cv2.imshow('image', right_img)
                # cv2.waitKey(500)
                cv2.imwrite('left/{}.png'.format(str(math.floor(timestamp))), left_img)
                # cv2.imwrite('/media/yuli/Extreme SSD/practice/left/{}.png'.format(str(math.floor(timestamp))), left_img)
                cv2.imwrite('right/{}.png'.format(str(math.floor(timestamp))), right_img)
                # cv2.imwrite('/media/yuli/Extreme SSD/practice/right/{}.png'.format(str(math.floor(timestamp))), right_img)
                print('image actually saved')
            except Exception as err:
                print(err)

    def save_image_left(self, data):
        print('left_image_received')
        if self.new_image[0] != 2:
            self.new_image[0] = 1
            self.left_image = data
            self.save_image()

    def save_image_right(self, data):
        print('right_image_received')
        if self.new_image[1] != 2:
            self.new_image[1] = 1
            self.right_image = data
            self.save_image()

def start_node():
    rospy.init_node('topic_to_png', anonymous=True)
    rospy.loginfo('topic_to_png node started')
    saveImage = SaveImage()
    rospy.Subscriber('/zed/zed_node/left/image_rect_color', Image, saveImage.save_image_left)
    rospy.Subscriber('/zed/zed_node/right/image_rect_color', Image, saveImage.save_image_right)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass