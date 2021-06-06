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

class Motion_predictor:
    def __init__(self):
        #step1: initialize node, publishers, subscribers
        rospy.init_node("motion_detector_predictor_node")
        self.bridge = CvBridge()
        self.pub1 = rospy.Publisher('camera/homography_image_hong', Image, queue_size=2)
        self.pub2 = rospy.Publisher('camera/homography_image_kong', Image, queue_size=2)
        self.pub3 = rospy.Publisher('camera/paper_filter_hong', Image, queue_size=2)
        self.pub4 = rospy.Publisher('camera/paper_filter_kong', Image, queue_size=2)
        self.pub5 = rospy.Publisher('camera/img_warp_hong', Image, queue_size=2)
        self.pub6 = rospy.Publisher('camera/img_warp_kong', Image, queue_size=2)
        self.pub7 = rospy.Publisher('camera/matching_point_image', Image, queue_size=2)
        self.pub8 = rospy.Publisher('cornerMatch/MidPoint',Point, queue_size=2)
        # self.sub1 = rospy.Subscriber("usb_cam_k/image_raw", Image, self.imageCallback1)
        self.br = tf.TransformBroadcaster()
        self.MidPoint = None

        #step2: initialize class predictor
        img_src = cv2.imread("/home/zach/catkin_ws/src/motion_detector/src/vision/cropped_sample/left0000.jpg")
        # creases = [[[-145,-145],[145,145]],[[0,0],[145,-145]]]
        creases = [[[145,-145],[-145,145]],[[-145,-145],[145,145]]]
        # img_src = cv2.imread("/home/zach/catkin_ws/src/motion_detector/src/vision/cropped_sample/airplane.png")
        # creases = [[[-210,-60],[0,150]],[[0,150],[210,-60]],[[-105,-150],[-105,45]],[[105,45],[105,-150]],[[0,-150],[0,150]]]
        pts_src = np.array([[-145, -145], [145, -145], [145, 145], [-145, 145]])
        result_img1 = copy.deepcopy(img_src)
        self.motion_detector1 = cl.Predictor(pts_src,creases,creases[0],result_img1)
        
        #step3: get paper's location via homography
        step=2
        state = 'state'+str(step)
        self.motion_detector1.get_facets_info(result_img1,0)
        self.motion_detector1.get_facets_info(result_img1,1)
        pts_src_location = self.motion_detector1.state[state]['contour_pts'] #could be generated by class Predictor
        print 'new pts src',pts_src_location
        # pts_src_location = [[0, 0], [290,0], [290, 290], [0, 290]]
        A = np.matrix([[741.2212917530331, 0, 311.8358797867751],
                       [0, 741.2317153584389, 240.6847621777156], [0.0, 0.0, 1.0]])  #intrinsic parameters of camera
        self.motion_detector2 = cl.GetTrans_new(pts_src_location,A)
        self.sub1 = rospy.Subscriber("usb_cam_h/image_raw", Image, self.imageCallback1)

        #step4: predict the grasp and match points
        grasp_point = copy.deepcopy(self.motion_detector1.state[state]['match_info']['grasp_pts_src'])
        grasp_point = grasp_point[0]
        # grasp_point = [grasp_point[0]+145,-grasp_point[1]+145]
        target_point = copy.deepcopy(self.motion_detector1.state[state]['match_info']['target_pts_src'])
        target_point = target_point[0]
        # target_point = [target_point[0]+145,-target_point[1]+145]
        self.motion_detector1.grasp_point = copy.deepcopy(grasp_point)
        self.motion_detector1.target_point = copy.deepcopy(target_point)
        self.sub2 = rospy.Subscriber("usb_cam_k/image_raw", Image, self.imageCallback2)

        rospy.spin()

    def imageCallback1(self,image):
        #broadcast the position of paper
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        clean_image = copy.deepcopy(cv_image)
        _,(Rotation, Translation), frame, filter_frame, img_warp=self.motion_detector2.detect_new(clean_image,view='top')

        #publish the images  
        if frame is not None:
            frame_pub = self.bridge.cv2_to_imgmsg(frame)
            self.pub1.publish(frame_pub)
        if filter_frame is not None:
            filter_pub = self.bridge.cv2_to_imgmsg(filter_frame)
            self.pub3.publish(filter_pub)
        if img_warp is not None:
            warp_pub = self.bridge.cv2_to_imgmsg(img_warp)
            self.pub5.publish(warp_pub)

        # clean_image0 = copy.deepcopy(cv_image)
        # tvecs=self.motion_detector1.detect_pnp(clean_image0,3)
        T = copy.deepcopy(Translation)
        # T = copy.deepcopy(tvecs)
        R = copy.deepcopy(Rotation)
        if T is not None:
            T = np.array(T)/1000
            real_height = T[2]
            error = abs(T[2])-real_height
            if abs(error)<0.1:
                if T[2] > 0:
                    trans_T = [T[0],T[1],T[2]]
                else:
                    T = -1*T
                    trans_T = [T[0],T[1],T[2]]
                # print 'T',trans_T 
                if abs(T[2]) <= 1:
                    quaternion = tf.transformations.quaternion_from_euler(R[0], R[1], R[2], axes='sxyz')
                    self.br.sendTransform(trans_T,quaternion,rospy.Time.now(),"paper","usb_cam1")

    def imageCallback2(self,image):
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        clean_image = cv_image.copy()
        is_blue=self.motion_detector2.detect_blue(clean_image) #if there is blue color, return 1
        #pt1: grasp point, pt2: target point. These pts can be generated by predictor
        pt1 = copy.deepcopy(self.motion_detector1.grasp_point)
        pt2 = copy.deepcopy(self.motion_detector1.target_point)
        # print 'pts src',self.motion_detector1.pts_src
        # print 'grasp point',self.motion_detector1.grasp_point
        # print 'target point',self.motion_detector1.target_point
        mid_point,result_img, homo_img, filter_frame, img_warp = self.motion_detector2.detect_mid_point(clean_image,pt1,pt2,view='side_left')

        #publish images
        if homo_img is not None:
            homo_pub = self.bridge.cv2_to_imgmsg(homo_img)
            self.pub2.publish(homo_pub)
        if filter_frame is not None:
            filter_pub = self.bridge.cv2_to_imgmsg(filter_frame)
            self.pub4.publish(filter_pub)
        if img_warp is not None:
            warp_pub = self.bridge.cv2_to_imgmsg(img_warp)
            self.pub6.publish(warp_pub)

        if mid_point is not None and is_blue==0:
            MidPoint = Point()
            MidPoint.x = mid_point[0]
            MidPoint.y = mid_point[1]
            MidPoint.z = mid_point[2]
            print 'mid point',mid_point
            self.MidPoint = MidPoint
            self.pub8.publish(MidPoint)

            image = self.bridge.cv2_to_imgmsg(result_img)
            self.pub7.publish(image)

        else:
            if self.MidPoint is not None:
                self.pub8.publish(self.MidPoint)

if __name__ == '__main__':
    detector = Motion_predictor()