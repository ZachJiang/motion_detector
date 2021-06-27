import rospy
import cv2
import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
import math
import shapeUtil as su
import time
from numpy.linalg import inv
import copy

from skimage import img_as_ubyte,img_as_float,exposure
from skimage.morphology import closing, square
from skimage.measure import label
from skimage.filters import threshold_otsu
from skimage.filters import threshold_yen
from skimage.exposure import rescale_intensity

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import Util as ut
from shapely.geometry import Polygon
from shapely.ops import cascaded_union,unary_union
import glob
kernel_elliptic_7 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
kernel_elliptic_15 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
area_threshold = 2000


class ColorFilter:
  def __init__(self):
    self.size=[1280,720]

  def detect(self,image):


    lower_black = np.array([0,0,0])  #-- Lower range --
    upper_black = np.array([70,70,70])  #-- Upper range --

    # red color boundaries [B, G, R]; lower = [1, 0, 20]; upper = [60, 40, 200]
    lower_red = np.array([1,0,20])  #-- Lower range --
    upper_red = np.array([40,40,255])  #-- Upper range --

    # lower_white = np.array([150,150,150])  #-- Lower range --
    # upper_white = np.array([255,255,255])  #-- Upper range --

    black_mask1 = cv2.inRange(image, lower_black, upper_black)
    kernel = np.ones((5,5),np.uint8)
    black_mask2 = cv2.dilate(black_mask1,kernel,iterations = 1)
    image[np.where(black_mask2 == [255])] = [160]

    # red_mask = cv2.inRange(image,lower_red,upper_red)
    # ret, thresh = cv2.threshold(red_mask, 50, 255, 0)
    # im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.drawContours(image, contours, -1, (0,255,0), 3)

    # white_mask1 = cv2.inRange(image, lower_white, upper_white)
    # image[np.where(white_mask1 == [0])] = [0]

    return image

  def paper_filter(self,image):
    #return paper mask
    imgHSV = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    imgG = cv2.cvtColor(imgHSV,cv2.COLOR_BGR2GRAY)
    img0 = cv2.morphologyEx(imgG,cv2.MORPH_OPEN,(11,11))
    imgC = cv2.morphologyEx(img0,cv2.MORPH_CLOSE,(11,11))
    imgC = cv2.GaussianBlur(imgC,(9,9),0)
    (_,imgC) = cv2.threshold(imgC,200,255,cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    imgC = cv2.bitwise_not(imgC)
    mask_paper = copy.deepcopy(imgC)
    # cm = CornerMatch_new()
    # mask_paper =cm.largestConnectComponent(imgC)
    mask = copy.deepcopy(mask_paper)
    mask[np.where(mask_paper == [255])] = [255]
    mask[np.where(mask_paper == [0])] = [0]
    # new_mask = cm.largestConnectComponent(mask)

    return mask

  def hsv_calc(self,frame):

        def nothing(x):
            pass

        cv2.namedWindow("Trackbars",)
        cv2.createTrackbar("lh","Trackbars",0,179,nothing)
        cv2.createTrackbar("ls","Trackbars",0,255,nothing)
        cv2.createTrackbar("lv","Trackbars",0,255,nothing)
        cv2.createTrackbar("uh","Trackbars",179,179,nothing)
        cv2.createTrackbar("us","Trackbars",255,255,nothing)
        cv2.createTrackbar("uv","Trackbars",255,255,nothing)
        while True:
            #frame = cv2.imread('candy.jpg')
            height, width = frame.shape[:2]
            #frame = cv2.resize(frame,(width/5, height/5), interpolation = cv2.INTER_CUBIC)
            hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

            lh = cv2.getTrackbarPos("lh","Trackbars")
            ls = cv2.getTrackbarPos("ls","Trackbars")
            lv = cv2.getTrackbarPos("lv","Trackbars")
            uh = cv2.getTrackbarPos("uh","Trackbars")
            us = cv2.getTrackbarPos("us","Trackbars")
            uv = cv2.getTrackbarPos("uv","Trackbars")

            l_blue = np.array([lh,ls,lv])
            u_blue = np.array([uh,us,uv])
            mask = cv2.inRange(hsv, l_blue, u_blue)
            result = cv2.bitwise_or(frame,frame,mask=mask)

            cv2.imshow("result",result)
            cv2.imshow("mask",mask)
            key = cv2.waitKey(1)
            #press esc to exit
            if key == 27:
                break
        cv2.destroyAllWindows()

class GetTrans_new:
    def __init__(self,pts_src,A):

        self.A = A
        self.pts_src = pts_src # reverse the order of the array
        self.motion_detector0 = ColorFilter()

    def detect_mid_point(self,frame,pt_src1,pt_src2,view):
        #detect the mid point of a line, used in corner match
        #step1: get the homography matrix
        frame0 = frame.copy()
        # print 'frame0 info',frame0.shape
        h_mat, (R,T), result_img1, filter_frame, img_warp= self.detect_new(frame0,view)
        self.mid_point = None
        print 'h_mat',h_mat

        #step2: get pt_dst1 and pt_dst2
        if h_mat is not None:
            pt_src1.append(1)
            pt_src2.append(1)
            pt_src1 = np.array(pt_src1)
            pt_src2 = np.array(pt_src2)
            # print 'pts1',pt_src1
            # print 'pts2',pt_src2
            pt_dst1 = self.transformReversePoints(pt_src1[0],pt_src1[1],h_mat)
            pt_dst2 = self.transformReversePoints(pt_src2[0],pt_src2[1],h_mat)

            #step3: get and draw the two points
            tar_point = [int(pt_dst2[0]),int(pt_dst2[1]),0]
            ori_point = [int(pt_dst1[0]),int(pt_dst1[1]),0]
            cv2.circle(frame, (int(tar_point[0]), int(tar_point[1])), 5, (0, 0, 255), 2)
            cv2.circle(frame, (int(ori_point[0]), int(ori_point[1])), 5, (0, 255, 255), 2)
            # cv2.circle(frame, (int(mid_persp[0]), int(mid_persp[1])), 10, (0, 255, 0), 2)
            self.tar_point = tar_point

            return tar_point,frame, result_img1, filter_frame, img_warp
        else:
            return None,frame,result_img1,filter_frame,None

    # Function to Detection Outlier on one-dimentional datasets.
    def delete_anomalies(self,data):
        #define a list to accumlate normal data
        new_data = []
        # print 'data',data

        data0 = np.array(data)[:,0]
        data1 = np.array(data)[:,1]
        data2 = np.array(data)[:,2]

        # Set upper and lower limit to 3 standard deviation
        data0 = np.nan_to_num(data0)
        data_std0 = np.std(data0)
        data_mean0 = np.mean(data0)
        anomaly_cut_off0 = data_std0 * 3
        lower_limit0 = data_mean0 - anomaly_cut_off0
        upper_limit0 = data_mean0 + anomaly_cut_off0

        data1 = np.nan_to_num(data1)
        data_std1 = np.std(data1)
        data_mean1 = np.mean(data1)
        anomaly_cut_off1 = data_std1 * 3
        lower_limit1 = data_mean1 - anomaly_cut_off1
        upper_limit1 = data_mean1 + anomaly_cut_off1

        data2 = np.nan_to_num(data2)
        data_std2 = np.std(data2)
        data_mean2 = np.mean(data2)
        anomaly_cut_off2 = data_std2 * 3
        lower_limit2 = data_mean2 - anomaly_cut_off2
        upper_limit2 = data_mean2 + anomaly_cut_off2

        # Generate inliers
        for i in range(len(data0)):
            d0 = data0[i]
            d1 = data1[i]
            d2 = data2[i]
            # print 'd0<',d0<upper_limit0
            # print 'd0>', d0>lower_limit0
            # print 'd1<',d1<upper_limit1
            # print 'd1>',d1>lower_limit1
            # print 'd2<',d2<upper_limit2
            # print 'd2>',d2>lower_limit2
            if d0 < upper_limit0 and d0 > lower_limit0 and d1 < upper_limit1 and d1 > lower_limit1 and d2 < upper_limit2 and d2 > lower_limit2:
                new_data.append([d0,d1,d2])

        trans0 = np.mean(np.array(new_data)[:,0])
        trans1 = np.mean(np.array(new_data)[:,1])
        trans2 = np.mean(np.array(new_data)[:,2])
        return [trans0,trans1,trans2]

    def detect_blue(self,image):
        #return 1 if there is blue color in the image
        image0 = copy.deepcopy(image)
        image1 = cv2.cvtColor(image0,cv2.COLOR_BGR2HSV)
        # lower_blue = np.array([70,150,0])  #-- Lower range --
        # upper_blue = np.array([132,205,176])  #-- Upper range --
        lower_blue = np.array([100,97,92])  #-- Lower range --
        upper_blue = np.array([169,193,165])  #-- Upper range --
        # lower_blue = (97,97,0)
        # upper_blue = (163,255,255)
        blue_mask1 = cv2.inRange(image1, lower_blue, upper_blue)
        mask_value = np.sum(blue_mask1)
        # print 'mask value',mask_value
        if mask_value<4000:
            return 0
        else:
            return 1

    def transformReversePoints(self,x,y,H0,reverse=False,integer=True):
            if reverse == False:
                H = H0
            else:
                val,H = cv2.invert(H0)
            # get the elements in the transform matrix
            h0 = H[0,0]
            h1 = H[0,1]
            h2 = H[0,2]
            h3 = H[1,0]
            h4 = H[1,1]
            h5 = H[1,2]
            h6 = H[2,0]
            h7 = H[2,1]
            h8 = H[2,2]
            tx = (h0*x + h1*y + h2)
            ty = (h3*x + h4*y + h5)
            tz = (h6*x + h7*y + h8)
            if integer==True:
                px = int(tx/tz)
                py = int(ty/tz)
                Z = int(1/tz)
            else:
                px = tx/tz
                py = ty/tz
                Z = 1/tz
            return (px,py)

    def detect_new(self, frame, view):
        # print "frame input info", frame.shape
        A = self.A
        pts_src = copy.deepcopy(self.pts_src)
        R, T = None, None
        im_perspCorr = None # black_image (300,300,3)   np.zeros((300,300,3), np.uint8)
        frame0 = frame.copy()
        # print 'frame0 imfo',frame0.shape
        # add object mask and detect contour
        filter_frame=self.motion_detector0.paper_filter(frame0)
        # print 'filter frame info',filter_frame.shape
        imgC = cv2.Canny(filter_frame, 50, 60)
        imgC = cv2.morphologyEx(imgC, cv2.MORPH_CLOSE, (3, 3))
        (cont, _)=cv2.findContours(imgC.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_approx = None
        lowest_error = float("inf")

        #contour selection
        for c in cont:
            pts_dst = []
            perim = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, .05 * perim, True) #the number affects the precision of contours
            area = cv2.contourArea(c)
            # print 'pts src in detect new',pts_src
            # print 'approx',approx

            if len(approx) == len(self.pts_src) and area>10000:
                right, error, new_approx = su.rightA(approx, 70,view) #80#change the thresh if not look vertically
                # print(right)
                new_approx = np.array(new_approx)
                # print 'new approx new',new_approx
                if error < lowest_error and right:
                    lowest_error = error
                    best_approx = new_approx
        #draw contour
        if best_approx is not None and frame is not None:
            cv2.drawContours(frame, [best_approx], 0, (255, 0, 0), 3)
            # print 'best approx',best_approx
            for i in range(0, len(best_approx)):
                pts_dst.append((best_approx[i][0][0], best_approx[i][0][1]))
                cv2.circle(frame, pts_dst[-1], 3+i, (i*30, 0, 255-i*20), 3)

            if len(pts_dst) < 4: #at least 4 points are needed (not co-linear points)
                # pts1 = pts_dst
                # pts2 = pts_src
                # M1 = cv2.getAffineTransform(np.float32(pts1),np.float32(pts2))
                # M1 = M1.tolist()
                # M1.append([0,0,1])
                # M1=np.array(M1)
                # print 'M1',M1
                # pt1 = self.transformReversePoints(pts_src[0][0],pts_src[0][1],M1,reverse=True)
                # pt2 = self.transformReversePoints(pts_src[1][0],pts_src[1][1],M1,reverse=True)
                # pt3 = self.transformReversePoints(pts_src[2][0],pts_src[2][1],M1,reverse=True)
                # pt4 = self.transformReversePoints((pts_src[0][0]+pts_src[1][0]+pts_src[2][0])/3,(pts_src[0][1]+pts_src[1][1]+pts_src[2][1])/3,M1,reverse=True)
                # cv2.circle(frame, (int(pt1[0]), int(pt1[1])), 10, (255, 255, 255), 2)
                # cv2.circle(frame, (int(pt2[0]), int(pt2[1])), 10, (255, 255, 255), 2)
                # cv2.circle(frame, (int(pt3[0]), int(pt3[1])), 10, (255, 255, 255), 2)
                # cv2.circle(frame, (int(pt4[0]), int(pt4[1])), 10, (255, 255, 255), 2)
                #modify pts_dst
                pts_dst = [pts_dst[0],pts_dst[1],pts_dst[2]]
                pts_src = [pts_src[0],pts_src[1],pts_src[2]]
                pts1 = pts_dst
                pts2 = pts_src
                M1 = cv2.getAffineTransform(np.float32(pts1),np.float32(pts2))
                M2 = cv2.getAffineTransform(np.float32(pts2),np.float32(pts1))
                frame0 = copy.deepcopy(frame)
                frame0 = cv2.warpAffine(frame0,M1,(290,290))
                new_dst_point = (int((1*pts_src[0][0]+0.7*pts_src[1][0]+1.3*pts_src[2][0])/3),int((1*pts_src[0][1]+0.7*pts_src[1][1]+1.3*pts_src[2][1])/3),1)
                # print 'new dst 0',new_dst_point
                new_dst_point = np.dot(M2,new_dst_point)
                # print 'new dst 1',new_dst_point
                new_dst_point = new_dst_point[:2]
                new_dst_point = new_dst_point.tolist()
                pts_dst.append(pts_dst[2])
                pts_dst[2] = new_dst_point
                #modify pts_src
                new_src_point = [int((pts_src[0][0]+pts_src[1][0]+pts_src[2][0])/3),int((pts_src[0][1]+pts_src[1][1]+pts_src[2][1])/3)]
                pts_src.append(pts_src[2])
                pts_src[2] = new_src_point

            #opencv findhomography
            # h, status = cv2.findHomography(np.array(pts_src).astype(float), np.array(pts_dst).astype(float),cv2.LMEDS,confidence=1)
            h, status = cv2.findHomography(np.array(pts_src).astype(float), np.array(pts_dst).astype(float))
            # h2 = su.H_from_points(np.array(pts_src1).astype(float), np.array(pts_dst).astype(float))
            # print 'status',status

            # get perspective corrected paper
            pts1 = pts_dst
            pts2 = pts_src
            # print 'pts dst',pts1
            # print 'pts src',pts2
            # img_warp = cv2.warpPerspective(frame0, h, (frame0.shape[1], frame0.shape[0]))
            # half_len = int(abs(pts_src[0][0]))
            # pts2 = pts_src + np.ones((4,2),dtype=int)*half_len
            M = cv2.getPerspectiveTransform(np.float32(pts1[:4]),np.float32(pts2[:4]))
            # img_size = (half_len*2, half_len*2)
            img_size = (290,290)
            im_perspCorr = cv2.warpPerspective(copy.deepcopy(frame),M,img_size)

            # show the homography result
            center = self.transformReversePoints(0,0,h)
            cv2.circle(frame, (int(center[0]), int(center[1])), 10, (0, 255, 255), 2)
            # p1 = self.transformReversePoints(pts_src[0][0],pts_src[0][1],h)
            # p2 = self.transformReversePoints(pts_src[1][0],pts_src[1][1],h)
            # p3 = self.transformReversePoints(pts_src[2][0],pts_src[2][1],h)
            # p4 = self.transformReversePoints(pts_src[3][0],pts_src[3][1],h)
            # cv2.circle(frame, (int(p1[0]), int(p1[1])), 10, (0, 255, 255), 2)
            # cv2.circle(frame, (int(p2[0]), int(p2[1])), 10, (0, 255, 255), 2)
            # cv2.circle(frame, (int(p3[0]), int(p3[1])), 10, (0, 255, 255), 2)
            # cv2.circle(frame, (int(p4[0]), int(p4[1])), 10, (0, 255, 255), 2)

            (R, T) = su.decHomography(A, h)
            ########liwei: change the decompose homography method and do one more transformation (from pixel frame to camera frame)
            num, Rs, Ts, Ns = cv2.decomposeHomographyMat(h, A)
            '''
            num possible solutions will be returned.
            Rs contains a list of the rotation matrix.
            Ts contains a list of the translation vector.
            Ns contains a list of the normal vector of the plane.
            '''
            Translation = Ts[0]
            # print 'num',num
            # print 'Ts',Ts
            # u0 = A[0,2]
            # v0 = A[1,2]
            # f = A[0,0]
            # Translation = [Ts[0][2]/f*(Ts[0][0]),Ts[0][2]/f*(Ts[0][1]),Ts[0][2]]
            # print 'R',R
            # print 'RS',Rs
            # print 'tranlation3',Translation
            Rot = su.decRotation(np.matrix(Rs[3]))
            ########liwei: change the decompose homography method and do one more transformation (from pixel frame to camera frame)

            zR = np.matrix([[math.cos(Rot[2]), -math.sin(Rot[2])], [math.sin(Rot[2]), math.cos(Rot[2])]])
            cv2.putText(imgC, 'rX: {:0.2f} rY: {:0.2f} rZ: {:0.2f}'.format(Rot[0] * 180 / np.pi, Rot[1] * 180 / np.pi, Rot[2] * 180 / np.pi), (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
            cv2.putText(imgC, 'tX: {:0.2f} tY: {:0.2f} tZ: {:0.2f}'.format(Translation[0][0], Translation[1][0], Translation[2][0]), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))


        if R is not None:
            Rotation = Rot
            # print 'detect T',T
            Translation = (T[0, 0], T[0, 1], T[0, 2])
            # print 'translation',Translation
            return h,(Rotation, Translation), frame, filter_frame, im_perspCorr
        else:
            return None, (None, None), frame, filter_frame, None

    def detect_pnp(self, frame, view):
        #step1: process image
        A = self.A
        pts_src = copy.deepcopy(self.pts_src)
        R, T = None, None
        frame0 = copy.deepcopy(frame)
        frame1=self.motion_detector0.paper_filter(frame0)
        imgC = cv2.Canny(frame1, 50, 60)
        imgC = cv2.morphologyEx(imgC, cv2.MORPH_CLOSE, (3, 3))
        (cont, _)=cv2.findContours(imgC.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # frame = cv2.drawContours(frame,cont,-1,(0,255,255),20)
        contour_img = self.bridge.cv2_to_imgmsg(copy.deepcopy(frame))
        self.pub3.publish(contour_img)
        best_approx = None
        lowest_error = float("inf")

        #step2: contour selection
        for c in cont:
            pts_dst = []
            perim = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, .01 * perim, True)
            area = cv2.contourArea(c)
            # print 'pts src',pts_src
            # print 'approx',approx

            if len(approx) == len(self.pts_src) and area>30000:
                right, error, new_approx = su.rightA(approx, 70,view) #80#change the thresh if not look vertically
                # print(right)
                new_approx = np.array(new_approx)
                # print 'new approx new',new_approx
                if error < lowest_error and right:
                    lowest_error = error
                    best_approx = new_approx

        if best_approx is not None and frame is not None:
            cv2.drawContours(frame, [best_approx], 0, (255, 0, 0), 3)

            for i in range(0, len(best_approx)):
                pts_dst.append((best_approx[i][0][0], best_approx[i][0][1]))
                cv2.circle(frame, pts_dst[-1], 3+i, (i*30, 0, 255-i*20), 3)

            if len(pts_dst) < 4: #at least 4 points are needed (not co-linear points)
                #modify pts_dst
                pts1 = pts_dst
                pts2 = pts_src
                M1 = cv2.getAffineTransform(np.float32(pts1),np.float32(pts2))
                M2 = cv2.getAffineTransform(np.float32(pts2),np.float32(pts1))
                frame0 = copy.deepcopy(frame)
                frame0 = cv2.warpAffine(frame0,M1,(290,290))
                new_dst_point = (int((1*pts_src[0][0]+0.7*pts_src[1][0]+1.3*pts_src[2][0])/3),int((1*pts_src[0][1]+0.7*pts_src[1][1]+1.3*pts_src[2][1])/3),1)
                # print 'new dst 0',new_dst_point
                new_dst_point = np.dot(M2,new_dst_point)
                # print 'new dst 1',new_dst_point
                new_dst_point = new_dst_point[:2]
                new_dst_point = new_dst_point.tolist()
                # new_dst_point = (int((pts_dst[0][0]+pts_dst[1][0]+pts_dst[2][0])/3),int((pts_dst[0][1]+pts_dst[1][1]+pts_dst[2][1])/3))
                pts_dst.append(pts_dst[2])
                pts_dst[2] = new_dst_point
                #modify pts_src
                new_src_point = [int((pts_src[0][0]+pts_src[1][0]+pts_src[2][0])/3),int((pts_src[0][1]+pts_src[1][1]+pts_src[2][1])/3)]
                pts_src.append(pts_src[2])
                pts_src[2] = new_src_point
        # print 'pts src',pts_src
        # print 'pts dst',pts_dst
        pts_src = [x + [0] for x in pts_src]
        pts_src = np.float32(pts_src)
        pts_dst = np.float32(pts_dst)

        #step3: get perspective points
        mtx = np.matrix([[741.2212917530331, 0, 311.8358797867751], [0, 741.2317153584389, 240.6847621777156], [0, 0, 1]])
        dist = np.matrix([[0.1145491812398767, -0.1410832391817006, -0.004166017347302669, 0.00185352697676791, 0]])
        ret,rvecs, tvecs = cv2.solvePnP(pts_src, pts_dst, mtx, dist,flags=cv2.SOLVEPNP_ITERATIVE)
        # print 'tvecs',tvecs
        # print 'rvecs',rvecs
        return tvecs

class GetCreases:
  def __init__(self):
    self.size=[1280,720]
  def detect(self,image):

    lower_black = np.array([0,0,0])  #-- Lower range --
    upper_black = np.array([70,70,70])  #-- Upper range --


    black_mask1 = cv2.inRange(image, lower_black, upper_black)
    kernel = np.ones((5,5),np.uint8)

    black_mask3 = cv2.dilate(black_mask1,kernel,iterations = 1)
    # black_mask3= cv2.GaussianBlur(black_mask2,(5,5),0)
  
    # imgC = cv2.Canny(black_mask2, 50, 60)
    # black_mask2 = cv2.morphologyEx(imgC, cv2.MORPH_CLOSE, (3, 3))
    # dim =  (600,600)
    # black_mask3 = cv2.resize(black_mask2, dim, interpolation = cv2.INTER_AREA)

    minLineLength = 40
    maxLineGap = 40 #250
    lines = cv2.HoughLinesP(black_mask3,1,np.pi/60,100,minLineLength,maxLineGap)
    edges_img = cv2.cvtColor(black_mask3,cv2.COLOR_GRAY2RGB)
    
    # if lines is not None:
    #   for line in lines:
    #     for x1,y1,x2,y2 in line:
    #       cv2.line(edges,(x1,y1),(x2,y2),(0,255,0),1)
    
    ##get post process result, and merge similar lines
    merged_lines = None
    if lines is not None:
      pp = HoughBundler()
      pp_result = pp.process_lines(lines, black_mask3)
      merged_lines = np.array(pp_result)

    if merged_lines is not None:
      for line in merged_lines:
        point1 = (line[0][0],line[0][1])
        point2 = (line[1][0],line[1][1])

        cv2.line(edges_img,point1,point2,(0,255,0),3)

    # print lines
    # print "merged_result", merged_lines

    return edges_img, merged_lines
  
class HoughBundler:
    '''Clasterize and merge each cluster of cv2.HoughLinesP() output
    a = HoughBundler()
    foo = a.process_lines(houghP_lines, binary_image)
    '''

    def get_orientation(self, line):
        '''get orientation of a line, using its length
        https://en.wikipedia.org/wiki/Atan2
        '''
        orientation = math.atan2((line[0] - line[2]), (line[1] - line[3]))
        return math.degrees(orientation)

    def checker(self, line_new, groups, min_distance_to_merge, min_angle_to_merge):
        '''Check if line have enough distance and angle to be count as similar
        '''
        for group in groups:
            # walk through existing line groups
            for line_old in group:
                # check distance
                if self.get_distance(line_old, line_new) < min_distance_to_merge:
                    # check the angle between lines
                    # print "###########distance", self.get_distance(line_old, line_new)

                    orientation_new = self.get_orientation(line_new)
                    orientation_old = self.get_orientation(line_old)
                    # print "#######angle", abs(orientation_new - orientation_old)
                    # if all is ok -- line is similar to others in group
                    if abs(orientation_new - orientation_old) < min_angle_to_merge:
                        group.append(line_new)
                        return False
        # if it is totally different line
        return True

    def merge_lines_pipeline_2(self, lines,min_distance_to_merge = 10,min_angle_to_merge = 10):
        'Clusterize (group) lines'
        groups = []  # all lines groups are here
        # Parameters to play with; original paras: 30, 30
        # min_distance_to_merge = 10
        # min_angle_to_merge = 10
        # first line will create new group every time
        groups.append([lines[0]])
        # if line is different from existing groups, create a new group
        for line_new in lines[1:]:
            if self.checker(line_new, groups, min_distance_to_merge, min_angle_to_merge):
                groups.append([line_new])

        return groups

    def merge_lines_segments1(self, lines):
        """Sort lines cluster and return first and last coordinates
        """
        orientation = self.get_orientation(lines[0])

        # special case
        if(len(lines) == 1):
            return [lines[0][:2], lines[0][2:]]

        # [[1,2,3,4],[]] to [[1,2],[3,4],[],[]]
        points = []
        for line in lines:
            points.append(line[:2])
            points.append(line[2:])
        # if vertical
        # if 45 < orientation < 135:
        if 84 < orientation < 96:
            #sort by y
            points = sorted(points, key=lambda point: point[1])
        else:
            #sort by x
            points = sorted(points, key=lambda point: point[0])

        # return first and last point in sorted group
        # [[x,y],[x,y]]
        return [points[0], points[-1]]

    def process_lines(self, lines, img, min_distance_to_merge = 10, min_angle_to_merge = 10):
        '''Main function for lines from cv.HoughLinesP() output merging
        for OpenCV 3
        lines -- cv.HoughLinesP() output
        img -- binary image
        '''
        lines_x = []
        lines_y = []
        # for every line of cv2.HoughLinesP()
        for line_i in [l[0] for l in lines]:
          
                orientation = self.get_orientation(line_i)
                # if vertical
                # if 45 < orientation < 135:
                if 84 < orientation < 96:
                    lines_y.append(line_i)
                else:
                    lines_x.append(line_i)

        lines_y = sorted(lines_y, key=lambda line: line[1])
        lines_x = sorted(lines_x, key=lambda line: line[0])
        merged_lines_all = []

        # for each cluster in vertical and horizantal lines leave only one line
        for i in [lines_x, lines_y]:
          # print "line_x", lines_x
          # print "line_y", lines_y
          if len(i) > 0:
            groups = self.merge_lines_pipeline_2(i, min_distance_to_merge, min_angle_to_merge)
            merged_lines = []
          
            for group in groups:
              merged_lines.append(self.merge_lines_segments1(group))

            merged_lines_all.extend(merged_lines)

        return merged_lines_all

    def distance_to_line(self, point, line):
      """Get distance between point and line
      https://stackoverflow.com/questions/40970478/python-3-5-2-distance-from-a-point-to-a-line
      """
      px, py = point
      x1, y1, x2, y2 = line
      x_diff = x2 - x1
      y_diff = y2 - y1
      num = abs(y_diff * px - x_diff * py + x2 * y1 - y2 * x1)
      den = math.sqrt(y_diff**2 + x_diff**2)
      return num / den

    def get_distance(self, a_line, b_line):
      """Get all possible distances between each dot of two lines and second line
      return the shortest
      """
      dist1 = self.distance_to_line(a_line[:2], b_line)
      dist2 = self.distance_to_line(a_line[2:], b_line)
      dist3 = self.distance_to_line(b_line[:2], a_line)
      dist4 = self.distance_to_line(b_line[2:], a_line)
      # print "#asjdfja",dist1, dist2, dist3, dist4
      return min(dist1, dist2, dist3, dist4)
        
class MatchFeatures:

    def SIFT(self, img_src):

        image,_ = self.filter(img_src,'paper')
        gray= cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        sift = cv2.xfeatures2d.SIFT_create()
        kp, des = sift.detectAndCompute(gray,None)
        # kp = sift.detect(gray,None)
        image=cv2.drawKeypoints(gray,kp,gray)
        # cv2.imshow('sift_keypoints.jpg',image)

        return image, kp
    
    def largestConnectComponent(self,bw_image):
        '''
        compute largest Connect component of an labeled image
        Parameters:
        ---
        bw_image:
            grey image in cv format
        Example:
        ---
            >>> lcc = largestConnectComponent(bw_img)
        '''
        bw_img = img_as_float(bw_image)
        thresh = threshold_otsu(bw_img)
        binary = bw_image > thresh

        labeled_img, num = label(binary, neighbors=4, background=0, return_num=True)    
        # plt.figure(), plt.imshow(labeled_img, 'gray')
        max_label = 0
        max_num = 0
        for i in range(1, num): # Start from 1 here to prevent the background from being set to the largest connected domain
            if np.sum(labeled_img == i) > max_num:
                max_num = np.sum(labeled_img == i)
                max_label = i
        lcc = (labeled_img == max_label)
        cv_image = img_as_ubyte(lcc)
        return cv_image

    def filter(self,image,color):
        blurr = cv2.GaussianBlur(image, (7, 7), 0)
        blurr_hsv = cv2.cvtColor(blurr, cv2.COLOR_BGR2HSV)
        #hsv color
        if color == 'green':
            # lowerG = (19,48,0)
            lowerG = (24,0,39)
            upperG = (96,255,255)
            maskG = cv2.inRange(blurr_hsv, lowerG, upperG)

            maskG = cv2.GaussianBlur(maskG, (5, 5), 0)
            mask = cv2.morphologyEx(maskG, cv2.MORPH_CLOSE, np.ones((19 ,19)))
            result = cv2.bitwise_and(blurr_hsv,blurr_hsv,mask=mask)
            result = cv2.cvtColor(result,cv2.COLOR_HSV2BGR)

        elif color == 'white':

            _,maskGW = self.filter(image, 'paper')
            lowerG = (24,0,39)
            upperG = (96,255,255)
            maskG = cv2.inRange(blurr_hsv, lowerG, upperG)
            maskNoG = cv2.bitwise_not(maskG)
            mask = cv2.bitwise_and(maskGW,maskNoG)

            # cv2.imshow('maskGW', maskGW)
            # cv2.imshow('maskW', mask)

            # mask = cv2.GaussianBlur(mask, (5, 5), 0)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((9 ,9)))            
            result = cv2.bitwise_or(blurr_hsv,blurr_hsv,mask=mask)
            result = cv2.cvtColor(result,cv2.COLOR_HSV2BGR) 
        elif color == 'red':
            lowerR = (0,119,0)
            upperR = (60,255,255)
            mask = cv2.inRange(blurr_hsv, lowerR, upperR)
            result = cv2.bitwise_or(blurr_hsv,blurr_hsv,mask=mask)
            result = cv2.cvtColor(result,cv2.COLOR_HSV2BGR) 
        elif color == 'blue':
            lowerB = (19,81,0)
            upperB = (168,255,255)

            kernel = np.ones((5,3),np.uint8)
            mask = cv2.inRange(blurr_hsv, lowerB, upperB)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((7 ,7)))
            mask = cv2.dilate(mask,kernel,iterations = 3)

            result = cv2.bitwise_or(blurr_hsv,blurr_hsv,mask=mask)
            result = cv2.cvtColor(result,cv2.COLOR_HSV2BGR)         
        elif color == 'paper':

            # lowerRB = (0, 85, 0)
            # upperRB = (179,255,255)

            lowerRB = (0, 69, 0)
            upperRB = (179,255,255)


            maskRB =  cv2.inRange(blurr_hsv, lowerRB, upperRB)
            mask = cv2.bitwise_not(maskRB)

            lcc = self.largestConnectComponent(mask)
            lcc = np.asarray(lcc, dtype="uint8")            
            
            mask = lcc
            result = cv2.bitwise_or(blurr_hsv,blurr_hsv,mask=mask)
            result = cv2.cvtColor(result,cv2.COLOR_HSV2BGR)            

        return result, mask

    def featureMatchingSIFT(self,img_src1,img_src2):
        MIN_MATCH_COUNT = 10

        image1,_ = self.filter(img_src1,'paper')
        image2,_ = self.filter(img_src2,'paper')

        img1 = cv2.cvtColor(image1,cv2.COLOR_BGR2GRAY)
        img2 = cv2.cvtColor(image2,cv2.COLOR_BGR2GRAY)

        sift = cv2.xfeatures2d.SIFT_create()
        kp1,des1 = sift.detectAndCompute(img1,None)
        kp2,des2 = sift.detectAndCompute(img2,None)

        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)

        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des1,des2,k=2)

        # store all the good matches as per Lowe's ratio test.
        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

        if len(good)>MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()
            # print img1.shape
            (h,w) = img1.shape
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            # dst = cv2.perspectiveTransform(pts,M)
            # img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)
        else:
            print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
            matchesMask = None

        draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                   singlePointColor = None,
                   matchesMask = matchesMask, # draw only inliers
                   flags = 2)

        img_comb = cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)
        
        return img_comb

    def featureMatchingORB(self,img_src1,img_src2):
        image1,_ = self.filter(img_src1,'paper')
        image2,_ = self.filter(img_src2,'paper')

        img1 = cv2.cvtColor(image1,cv2.COLOR_BGR2GRAY)
        img2 = cv2.cvtColor(image2,cv2.COLOR_BGR2GRAY)

        sift = cv2.ORB_create()
        kp1,des1 = sift.detectAndCompute(img1,None)
        kp2,des2 = sift.detectAndCompute(img2,None)

        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        matches = bf.match(des1,des2)
        matches = sorted(matches, key = lambda x:x.distance)

        draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                   singlePointColor = None,
                   matchesMask = None, # draw only inliers
                   flags = 2)

        img_comb = cv2.drawMatches(img1,kp1,img2,kp2,matches[:10],None,**draw_params)
        
        return img_comb

class ColorSegmentation:

    def kmeansColor(self, image, clusters=3, rounds=1):

        blur = cv2.blur(image,(5,5))
        blur0=cv2.medianBlur(blur,5)
        blur1= cv2.GaussianBlur(blur0,(5,5),0)
        blur2= cv2.bilateralFilter(blur1,9,75,75)  # make the image smooth
        image = cv2.cvtColor(blur2, cv2.COLOR_BGR2HSV)

        image1 = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

        h,w,_ = image.shape

        samples = np.zeros([h*w,3], dtype=np.float32)
        count = 0
        for x in range(h):
            for y in range(w):
                samples[count] = image[x][y]
                count += 1

        compactness, labels, centers = cv2.kmeans(samples,clusters,None,
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10000, 0.0001), 
                rounds, cv2.KMEANS_RANDOM_CENTERS)

        centers = np.uint8(centers)

        flat_labels = labels.flatten()

        mask_all = []
        sum_all = []
        for i in range(0,clusters):
            mask_i = np.array(flat_labels == i).astype(int)
            sum_i = np.sum(mask_i)
            mask_all.append(mask_i)
            sum_all.append(sum_i)
        
        print "sums", sum_all
        max_i =  np.argmax(sum_all, axis=0)
        print "i", max_i
        # mask_test = np.array([[0,255,0],[0,0,255],[255,0,0]])
        # print "image_test", mask_test

        res1 = centers[mask_all[max_i]]
        mask_background = 255*mask_all[max_i]
        mask_reshape = np.uint8(mask_background.reshape((image.shape[0],image.shape[1])))
        # print "mask_shape", mask_reshape.shape
        # print "mask",mask_reshape
        # print "image1",image1
        # print "image1_shape",image1.shape
        mask_reshape = np.bitwise_not(mask_reshape)
        mask = cv2.cvtColor(mask_reshape, cv2.COLOR_GRAY2BGR)

        return image1
    
    def goodFeaturesToTack(self,img):
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        corners = cv2.goodFeaturesToTrack(gray,50,0.005,20)
        corners = np.int0(corners)

        for i in corners:
            x,y = i.ravel()
            cv2.circle(img,(x,y),3,255,-1)
        return img
        
class topLayerMask:
    def GetMask(self,image,color):
        # image = cv2.resize(image,(640,480), interpolation = cv2.INTER_AREA)
        imgHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        imgG = cv2.cvtColor(imgHSV, cv2.COLOR_BGR2GRAY)
        # imgO = cv2.morphologyEx(imgG, cv2.MORPH_OPEN, (11, 11))
        # imgC = cv2.morphologyEx(imgO, cv2.MORPH_CLOSE, (11, 11))
        imgO = cv2.morphologyEx(imgG, cv2.MORPH_OPEN, (5, 5))
        imgC = cv2.morphologyEx(imgO, cv2.MORPH_CLOSE, (5, 5))
        # imgC = cv2.GaussianBlur(imgC,(9,9),0)

        (_, imgC) = cv2.threshold(imgG, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
       
        imgC = cv2.bitwise_not(imgC)
    
        # kernel = np.ones((5,5),np.uint8)
        # imgC = cv2.dilate(imgC,kernel,iterations = 1)

        mask_paper = self.largestConnectComponent(imgC)


        obj_img = cv2.bitwise_or(image,image,mask=mask_paper)

        _,mask_green = self.color_filter(obj_img,'green',mask_paper)
        _,mask_white = self.color_filter(obj_img,'white',mask_paper)
        _,mask_black = self.color_filter(obj_img,'black',mask_paper)
        _,mask_grass = self.color_filter(obj_img,'grass',mask_paper)
        _,mask_gray  = self.color_filter(obj_img,'gray',mask_paper)
        white_mask = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, np.ones((3 ,3)),iterations=3)

        
        _, tip_mask = self.color_filter(obj_img,color,mask_paper)

        merged_img = np.concatenate((obj_img,cv2.cvtColor(mask_green, cv2.COLOR_GRAY2BGR), cv2.cvtColor(mask_white, cv2.COLOR_GRAY2BGR), \
        cv2.cvtColor(mask_black, cv2.COLOR_GRAY2BGR),cv2.cvtColor(mask_grass, cv2.COLOR_GRAY2BGR),cv2.cvtColor(mask_gray, cv2.COLOR_GRAY2BGR)), axis=1)
        # ROI_mask = self.largestConnectComponent(ROI_mask)
        return merged_img,tip_mask
    
    def auto_canny(self,image, sigma=0.33):
        # compute the median of the single channel pixel intensities
        v = np.median(image)
        # apply automatic Canny edge detection using the computed median
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))
        edged = cv2.Canny(image, lower, upper)
        # return the edged image
        return edged

    def largestConnectComponent(self,bw_image):
        '''
        compute largest Connect component of an labeled image
        Parameters:
        ---
        bw_image:
            grey image in cv format
        Example:
        ---
            >>> lcc = largestConnectComponent(bw_img)
        '''
        bw_img = img_as_float(bw_image)
        thresh = threshold_otsu(bw_img)
        binary = bw_image > thresh

        labeled_img, num = label(binary, neighbors=4, background=0, return_num=True)    
        # plt.figure(), plt.imshow(labeled_img, 'gray')
        max_label = 0
        max_num = 0
        for i in range(1, num): # Start from 1 here to prevent the background from being set to the largest connected domain
            if np.sum(labeled_img == i) > max_num:
                max_num = np.sum(labeled_img == i)
                max_label = i
        
        lcc = (labeled_img == max_label)

        cv_image = img_as_ubyte(lcc)
        return cv_image
    
    def goodFeaturesToTack(self,gray_img):

        # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners = cv2.goodFeaturesToTrack(gray_img,6,0.001,30)
        corners = np.int64(corners)

        for i in corners:
            x,y = i.ravel()
            cv2.circle(gray_img,(x,y),6,255,-1)
        return gray_img
    
    def ROI_gripper(self,src_img,mask_img):
        lowerB = (97,97,0)
        upperB = (163,255,255)

        blurr = cv2.GaussianBlur(src_img, (3, 3), 0)
        blurr_hsv = cv2.cvtColor(blurr, cv2.COLOR_BGR2HSV)

        kernel = np.ones((29,29),np.uint8)
        mask = cv2.inRange(blurr_hsv, lowerB, upperB)
        canny_gripper = self.auto_canny(mask)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5 ,5)))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5 ,5)))
        mask = cv2.dilate(mask,kernel,iterations = 4)
        # cv2.imshow('ROI',mask)
        result = cv2.bitwise_and(mask_img,mask_img,mask=mask)
        
        canny_gripper = cv2.dilate(canny_gripper,np.ones((4,4),np.uint8),iterations=2)
        common = cv2.bitwise_and(canny_gripper,result)
        result = cv2.subtract(result,common)
        result = cv2.dilate(result,np.ones((3,3),np.uint8),iterations=3)
        return result,mask

    def color_filter(self,image,color,mask_paper):
        # image = cv2.fastNlMeansDenoisingColored(image,None,10,10,9,27)
        image_bgr = image.copy()
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #hsv color
        if color == 'green':
            # lowerG = (19,48,0)
            # lowerG = (0,0,45)
            # upperG = (179,68,63)
            # lowerG = (18,12,103)
            # upperG = (69,55,143)
            # lowerG = (33,1,85)
            # upperG = (107,36,128)
            # lowerG = (20,0,66)
            # upperG = (96,16,140)
            # lowerG = (58,0,91)
            # upperG = (87,45,255)
            # lowerG = (22,27,86)
            # upperG = (100,65,133)
            lowerG = (14,0,94)
            upperG = (85,44,116)
            maskW = cv2.inRange(image_hsv, lowerG, upperG)

            # cv2.imshow('maskGW', maskGW)
            # cv2.imshow('maskW', mask)

            # mask = cv2.GaussianBlur(mask, (5, 5), 0)
            mask = cv2.morphologyEx(maskW, cv2.MORPH_OPEN, np.ones((1 ,1)))            
            result = cv2.bitwise_or(image_bgr,image_bgr,mask=mask)    

        elif color == 'white':

            lowerW = (0,0,192)
            upperW = (179,255,255)
            mask = cv2.inRange(image_hsv, lowerW, upperW)

            # cv2.imshow('maskGW', maskGW)
            # cv2.imshow('maskW', mask)

            # mask = cv2.GaussianBlur(mask, (5, 5), 0)
            # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((1 ,1)))            
            result = cv2.bitwise_or(image_bgr,image_bgr,mask=mask)    

        elif color == 'black':

            # lowerW = (0,0,0)
            # upperW = (179,36,85)
            # lowerW = (0,0,33)
            # upperW = (83,82,93)
            # lowerW = (0,0,0)
            # upperW = (122,193,56)
            # lowerW = (38,9,0)
            # upperW = (87,255,93)
            lowerW = (0,0,0)
            upperW = (112,67,101)
            mask = cv2.inRange(image_hsv, lowerW, upperW)

            # cv2.imshow('maskGW', maskGW)
            # cv2.imshow('maskW', mask)

            # mask = cv2.GaussianBlur(mask, (5, 5), 0)
            # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((1 ,1)))
            mask = cv2.bitwise_and(mask,mask_paper)
            result = cv2.bitwise_or(image_bgr,image_bgr,mask=mask)    

        elif color == 'grass':

            # lowerG = (33,36,129)
            # upperG = (122,255,255)
            lowerG = (27,41,99)
            upperG = (76,255,255)
            maskW = cv2.inRange(image_hsv, lowerG, upperG)

            # cv2.imshow('maskGW', maskGW)
            # cv2.imshow('maskW', mask)

            # mask = cv2.GaussianBlur(mask, (5, 5), 0)
            mask = cv2.morphologyEx(maskW, cv2.MORPH_OPEN, np.ones((1 ,1)))            
            result = cv2.bitwise_or(image_bgr,image_bgr,mask=mask)    

        elif color == 'gray':

            # lowerW = (131,20,91)
            # upperW = (178,56,162)
            lowerW = (53,56,109)
            upperW = (157,71,255)
            # lowerW = (67,37,113)
            # upperW = (145,255,255)
            maskW = cv2.inRange(image_hsv, lowerW, upperW)

            # cv2.imshow('maskGW', maskGW)
            # cv2.imshow('maskW', mask)

            # mask = cv2.GaussianBlur(mask, (5, 5), 0)
            mask = cv2.morphologyEx(maskW, cv2.MORPH_OPEN, np.ones((1 ,1)))            
            result = cv2.bitwise_or(image_bgr,image_bgr,mask=mask)    

        return result, mask
        
    def avg_lines(self,image, lines):

        left = []
        right = []

        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)

            # Fit polynomial, find intercept and slope
            params = np.polyfit((x1, x2), (y1, y2), 1)
            slope = params[0]
            y_intercept = params[1]

            # print 'slope',slope
            # print 'y_intercept',y_intercept

            if slope < 0:
                left.append((slope, y_intercept)) #Negative slope = left lane
            else:
                right.append((slope, y_intercept)) #Positive slope = right lane

        # Avg over all values for a single slope and y-intercept value for each line

        left_avg = np.average(left, axis = 0)
        right_avg = np.average(right, axis = 0)

        # print 'lines',lines
        # print 'left',left_avg
        # print 'right',right_avg

        if len(left)==0 and len(right)==0:
            return np.array([])
        elif len(left)==0 and len(right)>0:
            right_line = self.get_coordinates(image, right_avg)
            return np.array([right_line])
        elif len(left)>0 and len(right)==0:
            left_line = self.get_coordinates(image, left_avg)
            return np.array([left_line])
        else:
            # Find x1, y1, x2, y2 coordinates for left & right lines
            left_line = self.get_coordinates(image, left_avg)
            right_line = self.get_coordinates(image, right_avg)
            return np.array([left_line, right_line])

    # Draws lines of given thickness over an image
    def draw_lines(self,image, lines1, lines2,thickness, color1, color2):

        # print(lines)
        line_image = np.zeros_like(image)
        # color=[0, 0, 255]


        if lines1 is not None:
            # print 'line',lines
            for x1, y1, x2, y2 in lines1:
                cv2.line(line_image, (x1, y1), (x2, y2), color1, thickness)
        if lines2 is not None:
            # print 'line',lines
            for x1, y1, x2, y2 in lines2:
                cv2.line(line_image, (x1, y1), (x2, y2), color2, thickness)

        # Merge the image with drawn lines onto the original.
        combined_image = cv2.addWeighted(image, 0.8, line_image, 1.0, 0.0)

        return combined_image

    def get_intersection_point(self,lines):
        #get two lines intersection point
        if lines is not None:
            if len(lines) == 2:
                intersection = su.line_intersect(lines[0][0],lines[0][1],
                                                 lines[0][2],lines[0][3],
                                                 lines[1][0],lines[1][1],
                                                 lines[1][2],lines[1][3])
                return intersection
            else:
                return None
        else:
            return None

    def get_coordinates(self,image, params):

        slope, intercept = params
        y1 = image.shape[0]
        y2 = int(y1 * (3/5)) # Setting y2 at 3/5th from y1
        x1 = int((y1 - intercept) / slope) # Deriving from y = mx + c
        x2 = int((y2 - intercept) / slope)

        if abs(slope) < 0.001:
            y1 = int(intercept)
            y2 = int(intercept)
            x1 = image.shape[1]
            x2 = int(x1 * (3/5))

        return np.array([x1, y1, x2, y2])


class Predictor:
    # predict the next state
    # 1) next pts_src (used in class GetTrans)
    # 2) next top color (used for class CornerMatch)
    # 3) cornerMatch information (point src, match type(point-point, point-line, l-p))

    #workflow: input detect pattern->get a) all creases; b) current crease->output a)grasp point; b) match point; c) contour (pts_src)
    #->go to input detect pattern
    def __init__(self,pts_src,creases,current_crease,original_image,step=0):

        self.pts_src = pts_src
        self.creases = creases
        self.current_crease = current_crease
        # crease direction is given by opencv
        self.state = {}
        self.original_image = original_image
        # print 'creases self',self.creases
        # print 'current crease',self.current_crease

        self.halfX = 205
        self.halfY = 205

    def crease_update(self,new_crease):
        # update crease info
        self.current_crease = new_crease

    def get_facets_info(self,image,step):
        # get all facets in the image, implemented by opencv polygon detection
        # get color information, implememted by self.get_colors
        # get corner match information, implemented by self.get_match_info
        print '##############predict'
        # print 'state crease',self.current_crease

        if step == 0:
            # get all facets information at the first step (get through image)
            facet_pts = {}
            facet_colors = {}
            image0 = copy.deepcopy(image)

            #step1: get facets contours (findContours)
            gray = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
            ret, binary = cv2.threshold(gray,127,255,cv2.THRESH_BINARY)
            contours, hierarchy = cv2.findContours(binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
            image0 = cv2.drawContours(image0,contours,-1,(0,0,255),3)
            # cv2.imshow('image0',image0)
            # cv2.waitKey(0)
            #test
            # for i in range(len(contours)):
            #     image_test = cv2.drawContours(image0,contours,i,(0,0,255),3)
            #     cv2.imshow('image1',image_test)
            #     cv2.waitKey(0)

            #step2: get and store points, color of each contour
            #transform from image frame to paper frame
            for i in range(0,len(contours)):
                perim = cv2.arcLength(contours[i], True)
                approx = cv2.approxPolyDP(contours[i], .05 * perim, True)
                new_approx = ut.frame_transform(approx,self.halfX,self.halfY)
                facet_pts.setdefault(str(i),new_approx)
                facet_colors.setdefault(str(i),0)

            #step3: get new contour information and construct state_dict
            contour_pts = copy.deepcopy(self.pts_src)
            contour_pts = contour_pts.tolist()
            state1 = {'facet_pts':copy.deepcopy(facet_pts),'facet_colors':copy.deepcopy(facet_colors),'contour_pts':contour_pts}
            self.state.setdefault('state1',state1)
            _,contour_img = self.get_new_contour(step)
            state1['contour_image'] = contour_img

            #step4: get match info and add it into the state dict
            match_info = self.get_match_info(step)
            self.state['state1']['match_info']=match_info
            self.state['state1']['crease'] = copy.deepcopy(self.current_crease)
            self.state['state1']['grasp_method'] = copy.deepcopy(self.get_grasp_method(step))
            # print 'state.match',self.state['state1']['match_info']
            # print 'state.contour',self.state['state1']['contour_pts']
            # print 'self.grasp_method',self.state['state1']['grasp_method']
            return image0

        else:
            #update the folded facets information (theoretically)

            #step1: find facets on the left side of the current state
            current_crease = copy.deepcopy(self.current_crease)
            state = 'state'+str(step)
            facet_pts = copy.deepcopy(self.state[state]['facet_pts'])
            left_facets,_ = ut.get_side_facets(current_crease,facet_pts)
            state1 = 'state'+str(step+1)
            facet_pts_new = copy.deepcopy(self.state[state]['facet_pts'])
            facet_colors_new = copy.deepcopy(self.state[state]['facet_colors'])

            #step2: reverse all points on the left facets, update color, match information
            for facet in facet_pts.keys():
                pts = copy.deepcopy(self.state[state]['facet_pts'][facet])
                colors =copy.deepcopy(self.state[state]['facet_colors'][facet])
                if facet in left_facets:
                    #reverse point and color+1
                    reversed_pts = []
                    for pt in pts:
                        reversed_pt = ut.reversePoint(current_crease,pt)
                        reversed_pts.append(reversed_pt)
                    colors = colors+1
                    facet_pts_new[facet]=reversed_pts
                    facet_colors_new[facet]=colors

            #step3: get new contour information and construct state_dict
            contour_pts, contour_img = self.get_new_contour(step)
            state_new = {'facet_pts':copy.deepcopy(facet_pts_new),'facet_colors':copy.deepcopy(facet_colors_new),'contour_pts':contour_pts, 'contour_image':contour_img}
            self.state.setdefault(state1,state_new)
            # print 'state new',state_new

            #step4: get match info and add it into the state dict
            self.crease_update(self.creases[step])
            match_info_new = self.get_match_info(step)
            self.state[state1]['match_info']=match_info_new
            self.state[state1]['crease'] = copy.deepcopy(self.current_crease)
            self.state[state1]['grasp_method'] = copy.deepcopy(self.get_grasp_method(step))
            # print 'state.match',self.state[state1]['match_info']
            # print 'state.contour',self.state[state1]['contour_pts']
            # print 'self.grasp_method',self.state[state1]['grasp_method']

            #step5: new contour image
            gray = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
            ret, binary = cv2.threshold(gray,127,255,cv2.THRESH_BINARY)
            contours, hierarchy = cv2.findContours(binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
            image = cv2.drawContours(image,contours,-1,(0,0,255),3)
            return image

    def get_new_contour(self,step):
        # get the new paper contour after folding (theory)

        #if step is 0, no need to do any further operations
        if step == 0:
            state = 'state'+str(step+1)
            pts = copy.deepcopy(self.state[state]['contour_pts'])
            pts_src_visuliaze = ut.frame_transform(copy.deepcopy(pts),self.halfX,self.halfY,inverse=1)
            mask=copy.deepcopy(self.original_image)
            mask[:]=(0,0,0)
            s=pts_src_visuliaze[0]
            e=pts_src_visuliaze[0]
            for p in pts_src_visuliaze:
                s=e
                e=p
                s = (int(s[0]),int(s[1]))
                e = (int(p[0]),int(p[1]))
                if s==e:
                    continue
                cv2.line(mask,s,e,(0,255,0),8)
            cv2.line(mask,e,(int(pts_src_visuliaze[0][0]),int(pts_src_visuliaze[0][1])),(0,255,0),8)
            return pts,mask
        current_crease = copy.deepcopy(self.current_crease)
        state = 'state'+str(step)
        pts = copy.deepcopy(self.state[state]['contour_pts'])
        a,b,c = ut.lineToFunction(current_crease)

        #step1: get pts at the left of the crease
        left_pts = []
        right_pts = []
        for pt in pts:
            product = a*pt[0]+b*pt[1]+c
            if product<0 and abs(product)>3000:
                left_pts.append(pt)
            elif product>0 and abs(product)>3000:
                right_pts.append(pt)
            elif abs(product)<=3000:
                left_pts.append(pt)
                right_pts.append(pt)
        left_pts.append(current_crease[1])
        left_pts.append(current_crease[0])
        right_pts.append(current_crease[1])
        right_pts.append(current_crease[0])    

        #step2: reverse the left pts
        reversed_left_pts = []
        for pt in left_pts:
            reversed_pt = ut.reversePoint(current_crease,pt)
            if reversed_pt[0]<-self.halfX:
                reversed_pt[0]=-self.halfX
            if reversed_pt[1]<-self.halfY:
                reversed_pt[1]=-self.halfY
            reversed_left_pts.append(reversed_pt)
        right_pts=ut.ccw(right_pts)
        reversed_left_pts=ut.ccw(reversed_left_pts)

        #step3: compare the two pts set and determine the new contour
        poly1 = Polygon(right_pts)
        poly2 = Polygon(reversed_left_pts)
        polygon = [poly1,poly2]
        contour = cascaded_union(polygon)
        new_pts_src = np.array(contour.exterior.coords)
        new_pts_src = new_pts_src.tolist()
        new_pts_src_visuliaze = ut.frame_transform(copy.deepcopy(new_pts_src),self.halfX,self.halfY,inverse=1)
        mask=copy.deepcopy(self.original_image)
        mask[:]=(0,0,0)
        s=new_pts_src_visuliaze[0]
        e=new_pts_src_visuliaze[0]
        for p in new_pts_src_visuliaze:
            s=e
            e=p
            s = (int(s[0]),int(s[1]))
            e = (int(p[0]),int(p[1]))
            if s==e:
                continue
            cv2.line(mask,s,e,(0,255,0),8)
        cv2.line(mask,e,(int(new_pts_src_visuliaze[0][0]),int(new_pts_src_visuliaze[0][1])),(0,255,0),8)

        #step4: re-arrange pts src
        new_pts_src=new_pts_src[:len(new_pts_src)-1]
        new_pts_src=ut.ccw(new_pts_src)

        #step5: delete points that are colinear (3 points in 1 line)
        new_pts_src1=copy.deepcopy(new_pts_src)
        for i in range(len(new_pts_src)):
            pt1=new_pts_src[i % len(new_pts_src)]
            pt2=new_pts_src[(i+1) % len(new_pts_src)]
            pt3=new_pts_src[(i+2) % len(new_pts_src)]
            l1=(pt2[0]-pt1[0],pt2[1]-pt1[1])
            l2=(pt3[0]-pt2[0],pt3[1]-pt2[1])
            angle=np.dot(l1,l2)/(np.linalg.norm(l1)*np.linalg.norm(l2))
            if angle==1:
                new_pts_src1.remove(pt2)
        
        ##test
        # cv2.imshow('mask',mask)
        # cv2.waitKey(0)

        return new_pts_src1,mask

    def get_new_contour_by_image(self,image):
        #this is to get contour of the image

        #step1: findContours
        gray = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
        ret, binary = cv2.threshold(gray,80,255,cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # image = cv2.drawContours(image,contours,len(contours)-1,(0,0,255),3)
        # print 'contour nums',len(contours)
        # cv2.imshow('image',image)

        #step2: approximate and store contour points
        perim = cv2.arcLength(contours[0], True)
        approx = cv2.approxPolyDP(contours[0], .05 * perim, True)
        new_approx = ut.frame_transform(approx,self.halfX,self.halfY)
        return new_approx

    def get_colors(self,step,reflect):
        #get colors for corner matching, return the upper color and lower color

        #step1: find facet on the right side of the crease (fixed facets)
        crease = copy.deepcopy(self.crease)
        state = 'state'+str(step+1)
        facet_pts = copy.deepcopy(self.state[state]['facet_pts'])
        _,right_facets = ut.get_side_facets(crease,facet_pts)

        #step2: get the color of lower paper (fixed paper)
        #assume there is only one color
        right_facet_colors = copy.deepcopy(self.state[state]['facet_colors'])
        color_max = 0
        for facet in right_facet_colors.keys():
            color = right_facet_colors[facet]
            if color>=color_max:
                color_max=color
        if color_max % 2 == 0:
            lower_color = 'white'
        else:
            lower_color = 'green'

        #step3: get the color of upper paper, this needs to be further modified
        if reflect == 0:
            upper_color = 'green'
        if reflect == 1:
            upper_color = 'white'

        return lower_color,upper_color

    def get_match_info(self,step):
        #get corner match information: 1) points src (grasp pt & match pt) for mathcing; 2) match type(p-p,p-l,l-p)

        #step1: find the grasp point src, the grasp point is assumed to be the furthest point from crease
        current_crease = copy.deepcopy(self.current_crease)
        state = 'state'+str(step+1)
        facet_pts = copy.deepcopy(self.state[state]['facet_pts'])
        a,b,c = ut.lineToFunction(current_crease)
        crease_func = [a,b,c]
        left_facets,right_facets = ut.get_side_facets(current_crease,facet_pts)
        grasp_point_info = ut.findFurthestPointInfo(crease_func,facet_pts,left_facets) #format:[point,distance]
        pts_src0 = np.array(grasp_point_info)[:,0] #grasp point src
        pts_src0 = pts_src0.tolist()

        #step2: find the target point src
        pts_src1 = []
        for pt in pts_src0:
            pt1 = ut.reversePoint(current_crease,pt)
            pts_src1.append(pt1)

        #step3: get the match type
        if len(pts_src0)==2:
            type = 'l-l'
        elif len(pts_src0)==1:
            #test if pts_src1[0] is on the facet_pts
            for facet in right_facets:
                pts = facet_pts[facet]
                type = 'p-l'
                # print 'corner match: pts',pts
                if ut.if_point_in_list(pts_src1[0],pts)==1:
                    type = 'p-p'

                    break
        else:
            type = 'unkown'

        match_info = {'grasp_pts_src':pts_src0,'target_pts_src':pts_src1,'match_type':type}

        return match_info

    def get_grasp_method(self,step):
        #determine the grasp method: flexflip or scoop
        #if the grasp point is located in more than 1 facet, then scoop; else flexflip

        #step1: get the facets and grasp information from previous states
        state = 'state'+str(step+1)
        facet_pts = copy.deepcopy(self.state[state]['facet_pts'])
        match_info = copy.deepcopy(self.state[state]['match_info'])
        grasp_point = copy.deepcopy(match_info['grasp_pts_src'])
        grasp_point = grasp_point[0]

        #step2: determine if the grasp point is located in more than 1 facet
        facet_polygons = []
        for facet in facet_pts.keys():
            facet_polygons.append(facet_pts[facet])
        is_in = ut.if_point_in_overlap_facets(grasp_point,facet_polygons,15,100)

        if is_in == 1:
            return 'scoop'
        elif is_in == 0:
            return 'flexflip'
        else:
            print '*************************grasp method error!'

class ParameterGenerator:
    #generate parameters for robotic execution function
    def updateState(self,state):
        self.state=state

    def analysis(self,step):
        print '%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%start patternAnalysis'
        #step1: get all information
        state_name = 'state'+str(step+1)
        current_state = copy.deepcopy(self.state[state_name])
        current_crease = copy.deepcopy(current_state['crease'])
        grasp_point = copy.deepcopy(current_state['match_info']['grasp_pts_src'])
        target_point = copy.deepcopy(current_state['match_info']['target_pts_src'])
        grasp_point=grasp_point[0]
        target_point=target_point[0]
        method = copy.deepcopy(current_state['grasp_method'])
        # print 'current crease',current_crease
        # print 'grasp point',grasp_point
        # print 'target point',target_point
        # print 'grasp method',method

        #step2: transform from paper frame to the world frame
        # rot_mat=np.matrix([[-1,0,0],[0,-1,0],[0,0,1]])
        rot_mat=np.matrix([[1,0,0],[0,1,0],[0,0,1]])
        grasp_point = ut.pointTransformation(grasp_point,[0,0,0],rot_mat)
        target_point = ut.pointTransformation(target_point,[0,0,0],rot_mat)
        crease_line = [current_crease[1][0]-current_crease[0][0],current_crease[1][1]-current_crease[0][1]]
        crease_norm = np.linalg.norm(crease_line)
        crease_axis = [float(crease_line[0]/crease_norm),float(crease_line[1]/crease_norm),0] #can be modify with vision
        crease_axis = ut.axisTransformation(crease_axis,rot_mat)
        crease_norm = float(crease_norm)/1000
        # grasp_point=grasp_point[0]
        # target_point=target_point[0]
        # crease_axis=crease_axis[0]
        crease_point = [float(current_crease[0][0]),float(current_crease[0][1]),0.0]
        crease_point = ut.pointTransformation(crease_point,[0,0,0],rot_mat)
        # crease_point=crease_point[0]
        # print 'grasp point',grasp_point
        # print 'crease_axis',crease_axis
        # print 'crease point',crease_point

        #step3: analyze the information
        trans_target2ref = [float(grasp_point[0])/1000,float(grasp_point[1])/1000,0]
        crease_perp_l = float(ut.pointsDistance(grasp_point,target_point)/1000)
        grasp_rot_angle = ut.findGraspAngle(method,crease_axis)
        startP2refP = [crease_point[0]/1000,crease_point[1]/1000,0]
        crease_rot_angle = ut.findMakeCreaseAngle(crease_axis)
        print 'trans_target2ref',trans_target2ref
        print 'crease axis',crease_axis
        print 'crease perp l',crease_perp_l
        print 'grasp rot angle',grasp_rot_angle
        print 'startP2refP',startP2refP
        print 'crease rot angle',crease_rot_angle
        return trans_target2ref,crease_axis,crease_perp_l,grasp_rot_angle,startP2refP,crease_rot_angle,method,crease_norm

class optical_flow:
    def __init__(self):

        self.feature_params = dict( maxCorners = 200,
                       qualityLevel = 0.001,
                       minDistance = 7,
                       blockSize = 7)

        # Parameters for lucas kanade optical flow
        self.lk_params = dict( winSize  = (15,15),
                        maxLevel = 2,
                        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        # Create some random colors
        self.color = np.random.randint(0,255,(100,3))
        self.count = 0
        self.old_gray = None
        self.p0 = None
        self.mask = None

    def get_optical_flow(self,frame,tip_mask,dir):
        # Take first frame and find corners in it


        tip_mask = cv2.dilate(tip_mask,np.ones((5,5),np.uint8),iterations=2)

        tip_mask = self.largestConnectComponent(tip_mask)

        tip_mask = cv2.erode(tip_mask,np.ones((8,8),np.uint8),iterations=1)

        tip_mask = cv2.dilate(tip_mask,np.ones((5,5),np.uint8),iterations=3)



        if self.count == 0:
                        
            self.old_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # (_, imgC) = cv2.threshold(self.old_gray, 225, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

            # self.old_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            p0_temp = cv2.goodFeaturesToTrack(self.old_gray, mask = None, useHarrisDetector=False, **self.feature_params)
            self.p0= []
            p0_fill =[]
            # print "p0_temp",p0_temp
            # Create a mask image for drawing purposes
            temp_figure = np.zeros_like(tip_mask)
            points = p0_temp
            for i,point in enumerate(points):
                a,b = point.ravel()
                # print 'a,b',a,b
                point_color = tip_mask[int(b),int(a)]
                if point_color >0:
                    p0_fill.append([[a,b]])
                    # print "p0_fill+++++++++++", p0_fill
                    
            p0_fill = np.array(p0_fill)

            if dir == 'max':
                temp_x_max = 0
                temp_y = 0
                for i,point in enumerate(p0_fill):
                    a,b = point.ravel()
                    if a > temp_x_max:
                        temp_x_max = a
                        temp_y = b
                print "=======",[temp_x_max,temp_y]
                self.p0 = np.array([[[temp_x_max,temp_y]]])
            elif dir == 'min':
                temp_x_min = 10000
                temp_y = 0
                for i,point in enumerate(p0_fill):
                    a,b = point.ravel()
                    if a < temp_x_min:
                        temp_x_min = a
                        temp_y = b
                print "=======",[temp_x_min,temp_y]
                self.p0 = np.array([[[temp_x_min,temp_y]]])

            self.mask = np.zeros_like(frame)
            self.count = self.count+1
            return None, None

        elif self.count > 0:
            ########start something new
            # print ret
            # frame_temp = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
            # frame_gray = cv2.cvtColor(frame_temp, cv2.COLOR_BGR2GRAY)

            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # calculate optical flow
            p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, frame_gray, self.p0, None, **self.lk_params)

            # Select good points
            if p1 is not None:
                good_new = p1[st==1]
                good_old = self.p0[st==1]
                print '========p0', self.p0[st==1]
                print '========p1', p1[st==1]
                # draw the tracks
                for i,(new,old) in enumerate(zip(good_new,good_old)):
                    a,b = new.ravel()
                    c,d = old.ravel()
                    self.mask = cv2.line(self.mask, (a,b),(c,d), self.color[i].tolist(), 2)
                    frame = cv2.circle(frame,(a,b),5,[0,255,0],-1)
                img = cv2.add(frame,self.mask)
                # img = frame
                merged_img = np.concatenate((img, cv2.cvtColor(tip_mask, cv2.COLOR_GRAY2BGR)), axis=1)

                # Now update the previous frame and previous points
                self.old_gray = frame_gray.copy()
                self.p0 = good_new.reshape(-1,1,2)
                return merged_img, self.p0[st==1]
            else:
                self.count = 0
                return None, None

    def largestConnectComponent(self,bw_image):
        '''
        compute largest Connect component of an labeled image
        Parameters:
        ---
        bw_image:
            grey image in cv format
        Example:
        ---
            >>> lcc = largestConnectComponent(bw_img)
        '''
        bw_img = img_as_float(bw_image)
        thresh = threshold_otsu(bw_img)
        binary = bw_image > thresh

        # binary = bw_image

        labeled_img, num = label(binary, neighbors=4, background=0, return_num=True)    
        # plt.figure(), plt.imshow(labeled_img, 'gray')
        max_label = 0
        max_num = 0
        for i in range(1, num+1): # Start from 1 here to prevent the background from being set to the largest connected domain

            if np.sum(labeled_img == i) > max_num:
                max_num = np.sum(labeled_img == i)
                max_label = i
        
        lcc = (labeled_img == max_label)

        cv_image = img_as_ubyte(lcc)
        return cv_image
