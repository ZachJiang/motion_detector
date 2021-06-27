#!/usr/bin/env python

import message_filters
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import math
import time
import classes as cl
import tf
import copy
from std_msgs.msg import UInt16

class Motion:
    def __init__(self):
        global start
        start = 0
        #step1: initialize node, publishers, subscribers
        rospy.init_node("motion_detector_node")
        self.bridge = CvBridge()
        self.pub7 = rospy.Publisher('camera/paper_filter_hong', Image, queue_size=2)
        self.pub8 = rospy.Publisher('camera/paper_filter_kong', Image, queue_size=2)
        self.pub1 = rospy.Publisher('camera/tip_mask',Image,queue_size=2)

        self.pub3 = rospy.Publisher('cornerMatch/vertexG',Point, queue_size=2)
        self.pub4 = rospy.Publisher('cornerMatch/vertexW',Point, queue_size=2)

        self.sub1 = rospy.Subscriber("usb_cam_k/image_raw", Image, self.imageCallback2)

        self.br = tf.TransformBroadcaster()
        self.MidPoint = None

        self.motion_detector3 = cl.topLayerMask()

        self.sub7 = rospy.Subscriber("cornerMatch/startOpticalFlow", UInt16,self.imageCallback7)
        self.pub6 = rospy.Publisher('camera/visible/opticalFlow', Image, queue_size=2)
        self.sub6 = rospy.Subscriber("usb_cam_k/image_raw", Image,self.imageCallback6)     
        
    
        self.bridge = CvBridge()
        self.motion_detector6 = cl.optical_flow()
        # self.tip_mask = None
        rospy.spin()


    def imageCallback2(self, image):

        #### get tip_mask for cornerMatch initialization
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        clean_image = cv_image.copy()

        step = rospy.wait_for_message('steps/getStep', UInt16).data
        color_list = ['green','white','white']
        dir_list = ['max','max','min']
        color = color_list[step]
        self.direction = dir_list[step]
        merged,tip_mask = self.motion_detector3.GetMask(clean_image,color)

        self.tip_mask = tip_mask
        image = self.bridge.cv2_to_imgmsg(tip_mask)
        self.pub1.publish(image)

    def imageCallback6(self,image):
        if start > 0:
            print "+++++++test2"
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            img,vertex_W=self.motion_detector6.get_optical_flow(cv_image,self.tip_mask,self.direction) 
            if img is not None:
                image = self.bridge.cv2_to_imgmsg(img)
                self.pub6.publish(image)
                vertexW = Point()
                vertexW.x = vertex_W[0][0]
                vertexW.y = vertex_W[0][1]
                vertexW.z = -1
                self.pub4.publish(vertexW)

    def imageCallback7(self,msg):
        # print "+++++++test1", int(msg.data)
        global start 
        start = int(msg.data)

if __name__ == '__main__':
    detector = Motion()
