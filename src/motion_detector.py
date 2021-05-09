#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import math
import time
import classes as cl

class Motion:
    def __init__(self):
        rospy.init_node("motion_detector_node")
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('camera/visible/image', Image, queue_size=2)
        rospy.Subscriber("usb_cam/image_raw", Image, self.imageCallback)
        # self.motion_detector1 = ColorFilter()
        self.motion_detector = cl.GetTrans()
        rospy.spin()

    def imageCallback(self, image):

        # if self.motion_detector2:
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        (R,T), result_img2 = self.motion_detector.detect(cv_image)
        image = self.bridge.cv2_to_imgmsg(result_img2)
        self.pub.publish(image)
        # self.pub.publish(image)



if __name__ == '__main__':
    detector = Motion()
