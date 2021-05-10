#!/usr/bin/env python

import rospy
import apriltag
import apriltag_ros
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

kernel_elliptic_7 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
kernel_elliptic_15 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
area_threshold = 2000

class MOG2:
  def __init__(self):
#    self.fgbg = cv2.BackgroundSubtractorMOG2(history=150, varThreshold=500, bShadowDetection=True)
# maybe we can try SubtractorKNN as well
    self.fgbg = cv2.createBackgroundSubtractorMOG2()
  def detect(self,image):
    fgmask = self.fgbg.apply(image)

    cv2.morphologyEx(fgmask, cv2.MORPH_CLOSE, kernel_elliptic_7, dst=fgmask)
    cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel_elliptic_15, dst=fgmask)

    contours = cv2.findContours(fgmask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#    area_box = ((cv2.contourArea(contour), cv2.boundingRect(contour)) for contour in contours[0])
    area_box = ((cv2.contourArea(contour), cv2.boundingRect(contour)) for contour in contours[1])
    area_box = [(area, box) for (area, box) in area_box if area > area_threshold]
    area_box.sort(reverse=True)

    bounding_boxes = [((x, y), (x+w, y+h)) for _, (x, y, w, h) in area_box[:5]]
    for p1, p2 in bounding_boxes:
        cv2.rectangle(image, p1, p2, (0, 255, 0), 2)

    return image
    #return fgmask #for param tuning

class Canny:
  def __init__(self):
    self.size=[1280,720]
  def detect(self,image):
    width=int(image.shape[1]/1)
    height=int(image.shape[0]/1)
    dim=(width,height)
    img=cv2.resize(image,dim,interpolation=cv2.INTER_AREA)
    image=cv2.Canny(img,50,150,apertureSize = 3)
    return image

class HarrisCorner:
  def __init__(self):
    self.size=[1280,720]
  def detect(self,image):
    width=int(image.shape[1]/1)
    height=int(image.shape[0]/1)
    dim=(width,height)
    img=cv2.resize(image,dim,interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # find Harris corners
    gray = np.float32(gray)
    dst = cv2.cornerHarris(gray,2,3,0.04)
    dst = cv2.dilate(dst,None)
    ret, dst = cv2.threshold(dst,0.01*dst.max(),255,0)
    dst = np.uint8(dst)

    # find centroids
    ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)
    # define the criteria to stop and refine the corners
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
    corners = cv2.cornerSubPix(gray,np.float32(centroids),(5,5),(-1,-1),criteria)
    # Now draw them
    res = np.hstack((centroids,corners))
    res = np.int0(res)
    img[res[:,1],res[:,0]]=[0,0,255]
    img[res[:,3],res[:,2]] = [0,255,0]
    return img

class ColorFilter:
  def __init__(self):
    self.size=[1280,720]
  def detect(self,image):
###method1:
    # lower = np.array([100,100,100])  #-- Lower range --
    # upper = np.array([200,200,200])  #-- Upper range --
    # mask = cv2.inRange(image, lower, upper)
    # res = cv2.bitwise_or(image, image, mask= mask)  #-- Contains pixels having the gray color--
    # return res
###method2:
    # img=image
    # black_pixels = np.where(
    # (img[:, :, 0] == 0) & 
    # (img[:, :, 1] == 0) & 
    # (img[:, :, 2] == 0))
    # img[black_pixels] = [255, 255, 255]
    # return img
###method3:
    width=int(image.shape[1]/2)
    height=int(image.shape[0]/2)
    dim=(width,height)
    image=cv2.resize(image,dim,interpolation=cv2.INTER_AREA)
    lower_white = np.array([100,100,100])  #-- Lower range --[140,140,140]
    upper_white = np.array([255,255,255])  #-- Upper range --
    lower_black = np.array([0,0,0])  #-- Lower range --
    upper_black = np.array([90,90,90])  #-- Upper range --
    rs=image
    black_mask = cv2.inRange(rs, lower_black, upper_black)
    rs[np.where(black_mask == [255])] = [255]
    white_mask = cv2.inRange(rs, lower_white, upper_white)
    rs[np.where(white_mask == [0])] = [0]
    #cv2.fastNlMeansDenoisingColored(rs,None,10,10,7,21)
    return rs
###method4:
    # width=int(image.shape[1]/3)
    # height=int(image.shape[0]/3)
    # dim=(width,height)
    # image=cv2.resize(image,dim,interpolation=cv2.INTER_AREA)
    # # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # # grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
  
    # # (thresh, image) = cv2.threshold(grayImage, 127, 255, cv2.THRESH_BINARY)
    # # image=cv2.cvtColor(image,cv2.COLOR_GRAY2RGB)

    # lower_white = np.array([100,100,100])  #-- Lower range --[140,140,140]
    # upper_white = np.array([255,255,255])  #-- Upper range --
    # lower_black = np.array([0,0,0])  #-- Lower range --
    # upper_black = np.array([100,100,100])  #-- Upper range --
    # rs=image

    # kernel = np.ones((4,4),np.uint8)
    # black_mask1 = cv2.inRange(image, lower_black, upper_black)
    
    
    # #black_mask1 = cv2.morphologyEx(black_mask1, cv2.MORPH_CLOSE, kernel)
    # black_mask1 = cv2.dilate(black_mask1,kernel,iterations = 1)
    # rs[np.where(black_mask1 == [255])] = [255]
    # white_mask1 = cv2.inRange(rs, lower_white, upper_white)
    # rs[np.where(white_mask1 == [0])] = [0]
    # cv2.fastNlMeansDenoisingColored(rs,None,10,10,7,21)
    # return rs

class Apriltag:
  #ref: https://www.pyimagesearch.com/2020/11/02/apriltag-with-python/
  def __init__(self):
    self.size=[1280,720] 
  def detect(self,image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(gray) 
    for r in results:
      (ptA, ptB, ptC, ptD) = r.corners
      ptB = (int(ptB[0]), int(ptB[1]))
      ptC = (int(ptC[0]), int(ptC[1]))
      ptD = (int(ptD[0]), int(ptD[1]))
      ptA = (int(ptA[0]), int(ptA[1]))
      # extract the bounding box (x, y)-coordinates for the AprilTag
      # and convert each of the (x, y)-coordinate pairs to integers

	    # draw the bounding box of the AprilTag detection
      cv2.line(image, ptA, ptB, (0, 255, 0), 2)
      cv2.line(image, ptB, ptC, (0, 255, 0), 2)
      cv2.line(image, ptC, ptD, (0, 255, 0), 2)
      cv2.line(image, ptD, ptA, (0, 255, 0), 2)
	    # draw the center (x, y)-coordinates of the AprilTag
      (cX, cY) = (int(r.center[0]), int(r.center[1]))
      cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
      # draw the estimated corner of the paper

    (cX_up, cY_up) = self.estimatCorner(results)
    cv2.circle(image, (cX_up, cY_up), 9, (255, 0, 0), -1)
    (cX_dw, cY_dw) = (int(r.center[0]), int(r.center[1]))
    cv2.circle(image, (cX_up, cY_up), 9, (255, 0, 0), -1)    
    # show the output image after AprilTag detection
    return image
  def estimatCorner(self,results):
    for r in results:
      (cX,cY)= (int(r.center[0]), int(r.center[1]))
      
    return a,b




class Contours:
  def __init__(self):
    self.size=[1280,720]
  def detect(self,image):
    width=int(image.shape[1]/1)
    height=int(image.shape[0]/1)
    dim=(width,height)
    img=cv2.resize(image,dim,interpolation=cv2.INTER_AREA)
    cv2.bilateralFilter(img,9,75,75)
    cv2.blur(img,(25,25))

    kernel = np.ones((9,9),np.float32)/81
    cv2.filter2D(img,-1,kernel)
    imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # ret, thresh = cv2.threshold(imgray, 127, 255, 0)
    ret, thresh = cv2.threshold(imgray, 50, 255, 0)
    im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, contours, -1, (0,255,0), 3)
    # cnt = contours[2]
    # cv2.drawContours(img, [cnt], 0, (0,255,0), 3)
    return img

class TagInfo:
  def __init__(self):
    self.size=[1280,720]
  def detect(self,image):
    width=int(image.shape[1]/1)
    height=int(image.shape[0]/1)
    dim=(width,height)
    img=cv2.resize(image,dim,interpolation=cv2.INTER_AREA)
    cv2.bilateralFilter(img,9,75,75)
    cv2.blur(img,(25,25))

    kernel = np.ones((9,9),np.float32)/81
    cv2.filter2D(img,-1,kernel)
    imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # ret, thresh = cv2.threshold(imgray, 127, 255, 0)
    ret, thresh = cv2.threshold(imgray, 50, 255, 0)
    im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, contours, -1, (0,255,0), 3)
    # cnt = contours[2]
    # cv2.drawContours(img, [cnt], 0, (0,255,0), 3)
    return img

class Motion:
    def __init__(self):
        rospy.init_node("motion_detector_node")
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('camera/visible/image', Image, queue_size=2)
        rospy.Subscriber("usb_cam/image_raw", Image, self.imageCallback)
        self.motion_detector1 = ColorFilter()
        # self.motion_detector2 = Canny()
        # self.motion_detector3 = HarrisCorner() 
        # self.motion_detector4 = Contours()
        self.motion_detector5 = Apriltag()
        rospy.Rate(10)
        rospy.spin()

    def imageCallback(self, image):

        if self.motion_detector1:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")

            # result_img1 = self.motion_detector1.detect(cv_image)
            # result_img4 = self.motion_detector4.detect(result_img1)
            # result_img3 = self.motion_detector3.detect(result_img4)
            #image = self.bridge.cv2_to_imgmsg(result_img, "8UC1")
            #image = self.bridge.cv2_to_imgmsg(result_img, "bgr8")
            result_img5 = self.motion_detector5.detect(cv_image)
            image = self.bridge.cv2_to_imgmsg(result_img5)
    
            
        self.pub.publish(image)

if __name__ == '__main__':
    detector = Motion()
