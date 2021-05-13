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
        self.pub1 = rospy.Publisher('camera/visible/image1', Image, queue_size=2)
        self.pub2 = rospy.Publisher('camera/visible/image2', Image, queue_size=2)

        # self.pub2 = rospy.Publisher()
        self.sub1 = rospy.Subscriber("usb_cam_h/image_raw", Image, self.imageCallback1)
        self.sub2 = rospy.Subscriber("usb_cam_k/image_raw", Image, self.imageCallback2)
        self.br = tf.TransformBroadcaster()
        
        self.motion_detector0 = cl.ColorFilter()

        # pts_src must be ccw ; center is [0,0]
        pts_src = np.array([[-145, -145], [145, -145], [145, 145], [-145, 145]])  #size of paper in real world, must be int && mm
        A = np.matrix([[741.2212917530331, 0, 311.8358797867751], 
                        [0, 741.2317153584389, 240.6847621777156], [0.0, 0.0, 1.0]])  #intrinsic parameters of camera
        self.motion_detector1 = cl.GetTrans(pts_src,A)
        self.motion_detector2 = cl.GetCreases()
        self.motion_detector3 = cl.CornerMatch()
        rospy.spin()

    def imageCallback1(self, image):

        # if self.motion_detector2:
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        clean_image = cv_image.copy()
        image0 = self.motion_detector0.detect(cv_image)
        R_mat, (R,T), result_img1, img_perspCorr = self.motion_detector1.detect(image0, clean_image)
        # print "img", img_perspCorr.shape
        

        result_img2 = None
        if img_perspCorr is not None:
            result_img2, crease_info = self.motion_detector2.detect(img_perspCorr)
            print 'crease info',crease_info


        if R is not None:
            if T[2] <= 1000 :
                quaternion = tf.transformations.quaternion_from_euler(R[0], R[1], R[2], axes='sxyz')
                T = (T[0]/1000,T[1]/1000,T[2]/1000)
                self.br.sendTransform(T,quaternion,rospy.Time.now(),"paper","usb_cam1")

                # for line in crease_info:
                #     p1 = line[0]
                #     p2 = line[1]
                #     p1.append(1)
                #     p2.append(1)
                #     rot_p1=np.dot(R,p1) + np.array(T)
                #     rot_p2=np.dot(R,p2) + np.array(T)

                # image = self.bridge.cv2_to_imgmsg(img_perspCorr,"bgr8")
                # self.pub.publish(image)

 
        if result_img2 is not None:
            image = self.bridge.cv2_to_imgmsg(result_img2)
        else: 
            image = self.bridge.cv2_to_imgmsg(result_img1)
        self.pub1.publish(image)

    def imageCallback2(self, image):

        # if self.motion_detector2:
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        clean_image = cv_image.copy()

        result_img3,vertex_w,vertex_g = self.motion_detector3.mainFuc(clean_image)

        image3 = self.bridge.cv2_to_imgmsg(result_img3)
        self.pub2.publish(image3)

if __name__ == '__main__':
    detector = Motion()
