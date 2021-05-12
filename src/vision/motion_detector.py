#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import math
import time
import classes as cl
import tf

class Motion:
    def __init__(self):
        rospy.init_node("motion_detector_node")
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('camera/visible/image', Image, queue_size=2)
        # self.pub2 = rospy.Publisher()
        rospy.Subscriber("usb_cam/image_raw", Image, self.imageCallback)
        
        self.br = tf.TransformBroadcaster()
        
        self.motion_detector0 = cl.ColorFilter()

        pts_src = np.array([[0, 0], [290, 0], [290, 290], [0, 290]])  #size of paper in real world, must be int && mm
        A = np.matrix([[741.2212917530331, 0, 311.8358797867751], 
                        [0, 741.2317153584389, 240.6847621777156], [0.0, 0.0, 1.0]])  #intrinsic parameters of camera
        self.motion_detector1 = cl.GetTrans(pts_src,A)
        self.motion_detector2 = cl.GetCreases()
        rospy.spin()

    def imageCallback(self, image):

        # if self.motion_detector2:
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        clean_image = cv_image.copy()
        image0 = self.motion_detector0.detect(cv_image)
        (R,T), result_img1, img_perspCorr = self.motion_detector1.detect(image0, clean_image)

        result_img2 = None
        if img_perspCorr is not None:
            result_img2 = self.motion_detector2.detect(img_perspCorr)
        
        
        if R is not None:
            if T[2] <= 1000 :
                quaternion = tf.transformations.quaternion_from_euler(R[0], R[1], R[2], axes='sxyz')
                T = (T[0]/1000,T[1]/1000,T[2]/1000)
                self.br.sendTransform(T,quaternion,rospy.Time.now(),"paper","usb_cam1")

                image = self.bridge.cv2_to_imgmsg(img_perspCorr,"bgr8")
                self.pub.publish(image)

                # if result_img2 is not None:
                #     image = self.bridge.cv2_to_imgmsg(result_img2)
                #     self.pub.publish(image)
        # self.pub.publish(image)



if __name__ == '__main__':
    detector = Motion()
