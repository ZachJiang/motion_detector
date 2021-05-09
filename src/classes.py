import rospy
import cv2
import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
import math
import shapeUtil as su
import time
from numpy.linalg import inv

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

    lower_white = np.array([100,100,100])  #-- Lower range --[140,140,140]
    upper_white = np.array([255,255,255])  #-- Upper range --
    lower_black = np.array([0,0,0])  #-- Lower range --
    upper_black = np.array([100,100,100])  #-- Upper range --
    rs=image

    kernel = np.ones((4,4),np.uint8)
    black_mask1 = cv2.inRange(image, lower_black, upper_black)

    rs[np.where(black_mask1 == [255])] = [255]
    white_mask1 = cv2.inRange(rs, lower_white, upper_white)
    rs[np.where(white_mask1 == [0])] = [0]
    rs[np.where(white_mask1 == [255])] = [255]
    cv2.fastNlMeansDenoisingColored(rs,None,10,10,7,21)
    return rs

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


class GetTrans:
    def __init__(self):

        self.A = np.matrix([[741.2212917530331, 0, 311.8358797867751], [0, 741.2317153584389, 240.6847621777156], [0.0, 0.0, 1.0]])
        self.red_point = (0, 0)
        self.red_lower = [115, 100, 100]
        self.red_upper = [125, 255, 255]
        pts_src = np.array([[0.0, 0.0], [297.0, 0.0], [297.0, 297.0],  # pixel measurements, just scaled ratio of sides
                            [0.0, 297.0]])

        # pts_src = pts_src / 1.05  # convert pixels to meters, can be changed for different sized "H"
        self.pts_src = pts_src[::-1]  # reverse the order of the array



    def detect(self, frame):

        #global out
        A = self.A
        pts_src = self.pts_src
        R, T = None, None
        blurr = cv2.GaussianBlur(frame, (5, 5), 0)
        imgG = cv2.cvtColor(blurr, cv2.COLOR_BGR2GRAY)
        imgC = cv2.Canny(imgG, 50, 60)
        imgC = cv2.morphologyEx(imgC, cv2.MORPH_CLOSE, (3, 3))
        # imgC = cv2.dilate(imgC, (3, 3), iterations=2)
        # (_,cont, _) = cv2.findContours(imgC.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        (_,cont, _)=cv2.findContours(imgC.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_approx = None
        lowest_error = float("inf")

        #contour selection
        for c in cont:
            pts_dst = []
            perim = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, .01 * perim, True)
            area = cv2.contourArea(c)

            if len(approx) == len(self.pts_src):
                right, error = su.rightA(approx, 80) #change the thresh if not look vertically
                # print(right)
                if error < lowest_error and right:
                    lowest_error = error
                    best_approx = approx

        # red_point, _ = su.detectColor(blurr, red_lower, red_upper)
        # if red_point is not None:
           # cv2.circle(frame, (red_point[0] + frame.shape[0] / 2, red_point[1] + frame.shape[1] / 2), 5, (0, 0, 255), 2)

        if best_approx is not None:
            # print 'best approx',best_approx
            # print 'red point',self.red_point
            cv2.drawContours(frame, [best_approx], 0, (255, 0, 0), 3)

            for i in range(0, len(best_approx)):
                pts_dst.append((best_approx[i][0][0], best_approx[i][0][1]))
                # cv2.circle(frame, pts_dst[-1], 3, (i*30, 0, 255-i*20), 3)

            # Correction method for contour points.  Need to make sure the points are mapped correctly
            pts_dst = su.sortContour(
                np.array((self.red_point[0] + pts_dst[0][0], self.red_point[1] + pts_dst[0][1])), pts_dst)

            cv2.circle(frame, pts_dst[0], 7, (0, 255, 0), 4)
            cv2.circle(frame, pts_dst[1], 7, (200, 0, 200), 4)
            # print 'pts',pts_dst

            center = su.line_intersect(pts_dst[0][0],pts_dst[0][1],pts_dst[2][0],pts_dst[2][1],
                                         pts_dst[1][0],pts_dst[1][1],pts_dst[3][0],pts_dst[3][1])
            # print 'center',center
            cv2.circle(frame, (int(center[0]), int(center[1])), 5, (0, 0, 255), 2)

            for i in range(0, len(best_approx)):
                cv2.circle(frame, pts_dst[i], 3, (i * 30, 0, 255 - i * 20), 3)

            h, status = cv2.findHomography(np.array(pts_src).astype(float), np.array(pts_dst).astype(float))            # print 'h',h
            # print 'status',status
            #warped = cv2.warpPerspective(base, h, (base.shape[1], base.shape[0]))


            (R, T) = su.decHomography(A, h)
            Rot = su.decRotation(R)
            # print 'R',R
            # print 'T',T
            # center = center - np.array(T)
            # inv_R = inv(np.matrix(R))
            # center = center.reshape(3,1)
            # center = np.dot(inv_R,center)
            #
            # print 'center',center

            zR = np.matrix([[math.cos(Rot[2]), -math.sin(Rot[2])], [math.sin(Rot[2]), math.cos(Rot[2])]])
            cv2.putText(imgC, 'rX: {:0.2f} rY: {:0.2f} rZ: {:0.2f}'.format(Rot[0] * 180 / np.pi, Rot[1] * 180 / np.pi, Rot[2] * 180 / np.pi), (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
            cv2.putText(imgC, 'tX: {:0.2f} tY: {:0.2f} tZ: {:0.2f}'.format(T[0, 0], T[0, 1], T[0, 2]), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
            pDot = np.dot((-200, -200), zR)
            # print 'rot',Rot
            # print 'zr',zR
            # pDot = np.dot((-148, -148),zR)
            self.red_point = (int(pDot[0, 0]), int(pDot[0, 1]))

            # cv2.circle(frame, (int(pDot[0, 0]) + pts_dst[0][0], int(pDot[0, 1]) + pts_dst[0][1]), 5, (0, 0, 255), 2)

        merged_img = np.concatenate((frame, cv2.cvtColor(imgC, cv2.COLOR_BAYER_GB2BGR)), axis=1)

        if R is not None:
            # print 'R',R
            # print 'T',T
            return (R, T), merged_img
        else:
            return (R, T), merged_img

