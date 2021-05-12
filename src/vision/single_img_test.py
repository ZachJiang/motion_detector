#!/usr/bin/env python

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import math
import time
import classes as cl
import tf


if __name__ == '__main__':


###test 1: GetCreeses
 ## For one image
    # # Read the image.
    # img_src = cv2.imread("cropped_sample/left0000.jpg")

    # motion_detector1 = cl.GetCreases()
    # result_img1, creases = motion_detector1.detect(img_src)  
    # # Show output
    # # cv2.imshow("Original_image", img_src)
    # cv2.imshow("Image", result_img1)
    # cv2.waitKey(0)

 # For all images
    # for i in range(193):
    #     print 'i',i
    #     img_src = cv2.imread('cropped_sample/left'+"{0:0>4}".format(i)+'.jpg')
    #     print 'cropped_sample/left'+"{0:0>4}".format(i)+'.jpg'
        
    #     motion_detector1 = cl.GetCreases()
    #     result_img1, creases = motion_detector1.detect(img_src)  
    #     # Show output
    #     # cv2.imshow("Original_image", img_src)
    #     cv2.imshow("Image", result_img1)
    #     cv2.waitKey(0)
        
###test 2: CornerMatch
 ##For one image:

    # img_src = cv2.imread('corner_match_sample/left0023.jpg')
    # motion_detector = cl.CornerMatch()
    # #step1: image process and canny
    # result_img = motion_detector.detect(img_src)
    # # cv2.imshow("result_img", result_img)

    # #step2: roi mask
    # result_img2 = motion_detector.ROI_mask(result_img)
    # # cv2.imshow('result_img2',result_img2)

    # #step3: houghline transform
    # lines = cv2.HoughLinesP(result_img2,
    #                         rho=2,              #Distance resolution in pixels
    #                         theta=np.pi / 180,  #Angle resolution in radians
    #                         threshold=60,      #Min. number of intersecting points to detect a line
    #                         lines=np.array([]), #Vector to return start and end points of the lines indicated by [x1, y1, x2, y2]
    #                         minLineLength=2,   #Line segments shorter than this are rejected
    #                         maxLineGap=25       #Max gap allowed between points on the same line
    #                         )
    # # print 'lines',lines

    # averaged_lines = motion_detector.avg_lines(img_src, lines)              #Average the Hough lines as left or right lanes
    # combined_image = motion_detector.draw_lines(img_src, averaged_lines, 5)

    # cv2.imshow('image',combined_image)
    # cv2.waitKey(0)

 ##For all images:

    for i in range(0,277):
        # print 'i',i
        img_src = cv2.imread('corner_match_sample/left'+"{0:0>4}".format(i)+'.jpg')
        print 'str','corner_match_sample/left'+"{0:0>4}".format(i)+'.jpg'
        motion_detector = cl.CornerMatch()

        image,white_vertex,green_vertex = motion_detector.mainFuc(img_src)
        cv2.imshow("corner matchf",image)        
        print 'white vertex',white_vertex
        print 'green vertex',green_vertex

        cv2.waitKey(0)

