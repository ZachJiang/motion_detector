#!/usr/bin/env python

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import math
import time
import classes as cl
import tf
import copy
from skimage import img_as_ubyte,img_as_float,exposure



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
   #  for i in range(193):
   #      print 'i',i
   #      img_src = cv2.imread('cropped_sample/left'+"{0:0>4}".format(i)+'.jpg')
   #      print 'cropped_sample/left'+"{0:0>4}".format(i)+'.jpg'
   #      motion_detector1 = cl.GetCreases()
   #      result_img1, creases = motion_detector1.detect(img_src)  
   #      # Show output
   #      # cv2.imshow("Original_image", img_src)
   #      print "crease info", creases
   #      cv2.imshow("Image", result_img1)
   #      cv2.waitKey(0)
        
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

   #For all images:

   #  for i in range(0,277):
   #      # print 'i',i
   #      img_src = cv2.imread('corner_match_sample/left'+"{0:0>4}".format(i)+'.jpg')
   #      print 'str','corner_match_sample/left'+"{0:0>4}".format(i)+'.jpg'
   #      motion_detector = cl.CornerMatch()
   #      image,white_vertex,green_vertex = motion_detector.mainFuc(img_src)
   #      cv2.imshow("corner match test",image)        
   #      print 'white vertex',white_vertex
   #      print 'green vertex',green_vertex
   #      cv2.waitKey(0)

###test 3: CornerMatch_new:
 ##For 1 image:

      #   i=50
      #   img_src = cv2.imread('diag_fold_sample2/left'+"{0:0>4}".format(i)+'.jpg')
      #   print 'str','corner_match_sample3/left'+"{0:0>4}".format(i)+'.jpg'
      #   motion_detector = cl.CornerMatch_new()
      # #   skimage_src = img_as_float(img_src)
      # #   skimage = exposure.equalize_adapthist(skimage_src, clip_limit=0.01)
      # #   img_src = img_as_ubyte(skimage)
      #   image,white_vertex,green_vertex = motion_detector.hsv_calc(img_src)
      #   cv2.imshow("corner match",image)        
      #   print 'white vertex',white_vertex
      #   print 'green vertex',green_vertex
      #   cv2.waitKey(0)

 ##For all images:

   #  for i in range(0,500):
   #      # print 'i',i
   #      img_src = cv2.imread('diag_fold_sample2/left'+"{0:0>4}".format(i)+'.jpg')
   #      print 'str','imgs_temp/left'+"{0:0>4}".format(i)+'.jpg'
   #      motion_detector = cl.CornerMatch_new()


   #      image,white_vertex,green_vertex = motion_detector.mainFuc(img_src)
   #      cv2.imshow("corner match",image)        
   #      print 'white vertex',white_vertex
   #      print 'green vertex',green_vertex

   #      cv2.waitKey(0)

###test 4: MatchFeatures:
 ##For 1 image:

      #   i=1
      #   img_src1 = cv2.imread('corner_match_sample3/left'+"{0:0>4}".format(i)+'.jpg')
      #   print 'str','corner_detect_img/left'+"{0:0>4}".format(i)+'.jpg'

      #   i=680
      #   img_src2 = cv2.imread('corner_match_sample3/left'+"{0:0>4}".format(i)+'.jpg')
      #   print 'str','corner_detect_img/left'+"{0:0>4}".format(i)+'.jpg'

      #   motion_detector = cl.MatchFeatures()
      # #   skimage_src = img_as_float(img_src)
      # #   skimage = exposure.equalize_adapthist(skimage_src, clip_limit=0.01)
      # #   img_src = img_as_ubyte(skimage)
      #   image = motion_detector.featureMatching(img_src1,img_src2)
      #   cv2.imshow("featureMatching",image)        
      #   cv2.waitKey(0)

 ##For all images:

      #   for index in range(1,191):
      #    i = index*3
      #    img_src1 = cv2.imread('corner_match_sample6/left'+"{0:0>4}".format(i)+'.jpg')
      #    print 'str','corner_detect_img/left'+"{0:0>4}".format(i)+'.jpg'
      #    j=i+3
      #    img_src2 = cv2.imread('corner_match_sample6/left'+"{0:0>4}".format(j)+'.jpg')
      #    print 'str','corner_detect_img/left'+"{0:0>4}".format(j)+'.jpg'

      #    motion_detector = cl.MatchFeatures()

      #    image = motion_detector.featureMatchingORB(img_src1,img_src2)
      #    cv2.imshow("featureMatching",image)        
      #    cv2.waitKey(0)

      #   cv2.destroyAllWindows
      #   exit()

###test 5: ColorSegmentation:

      # for i in range(100,1000):
      #    # img_src= cv2.imread('diag_fold_sample2/left'+"{0:0>4}".format(i)+'.jpg')
      #    img_src= cv2.imread('diag_fold_sample2/left'+"{0:0>4}".format(i)+'.jpg')      
      #    print 'str','corner_match_sample2/left'+"{0:0>4}".format(i)+'.jpg'

      #    motion_detector = cl.ColorSegmentation()
      #    #   skimage_src = img_as_float(img_src)
      #    #   skimage = exposure.equalize_adapthist(skimage_src, clip_limit=0.01)
      #    #   img_src = img_as_ubyte(skimage)

      #    # image = motion_detector.kmeansColor(img_src,clusters=3,rounds=5)
      #    ## Shi-Tomasi method
      #    image = motion_detector.goodFeaturesToTack(img_src)

      #    cv2.imshow("goodFeature",image)        
      #    cv2.waitKey(0)

###test 6: CornerMatch_v3:

      # for i in range(1,1000):
      #    # img_src= cv2.imread('diag_fold_sample5/left'+"{0:0>4}".format(i)+'.jpg')
      #    img_src= cv2.imread('corner_match_sample3/left'+"{0:0>4}".format(i)+'.jpg')      
      #    print 'str','corner_match_sample2/left'+"{0:0>4}".format(i)+'.jpg'

      #    motion_detector1 = cl.CornerMatch_v3()
      #    #   skimage_src = img_as_float(img_src)
      #    #   skimage = exposure.equalize_adapthist(skimage_src, clip_limit=0.01)
      #    #   img_src = img_as_ubyte(skimage)

      #    # image = motion_detector.kmeansColor(img_src,clusters=3,rounds=5)
      #    ## Shi-Tomasi method
      #    image,_,_ = motion_detector1.GetEdges(img_src)

      #    # cv2.imshow("CornerMatch_v2",image)        
      #    cv2.waitKey(0)


##test 5: Predictor
 # For one image

   #step1: get segmented image (segmented by creases).
   pts_src0 = np.array([[0, 0], [290, 0], [290, 290], [0, 290]])
   # img_src = cv2.imread("cropped_sample/airplane.png")
   img_src = cv2.imread("cropped_sample/left0000.jpg")
   # img_src = cv2.resize(img_src, (420,300), interpolation = cv2.INTER_AREA)
   creases = [[[-145,-145],[145,145]],[[0,0],[145,-145]]]
   # creases = [[[-210,-60],[0,150]],[[0,150],[210,-60]],[[-105,-150],[-105,45]],[[105,45],[105,-150]],[[0,-150],[0,150]]]

   # creases = [[[0,290],[290,0]],[[0,0],[290,290]]]
   # cv2.imshow("Original_image", img_src)
   # cv2.imshow("Image", result_img1)

   #step2: detect polygons
   # pts_src = np.array([[-210, -150], [210, -150], [210, 150], [-210, 150]])
   pts_src = np.array([[-145, -145], [145, -145], [145, 145], [-145, 145]])
   result_img1 = copy.deepcopy(img_src)
   motion_detector2 = cl.Predictor(pts_src,creases,creases[0],result_img1)
   motion_detector2.get_facets_info(result_img1,0)
   # motion_detector2.crease_update(creases[1])
   motion_detector2.get_facets_info(result_img1,1)
   # motion_detector2.get_facets_info(result_img1,2)
   # motion_detector2.get_facets_info(result_img1,3)
   # motion_detector2.get_facets_info(result_img1,4)
   

   cv2.waitKey(0)