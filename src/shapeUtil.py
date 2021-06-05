#!/usr/bin/env python
import cv2
import numpy as np
import math
import copy
# Gets passed an approx contour list and finds if all the angles are 90 degrees

def rightA_old(approx, thresh, side_view):
    right = True
    error = 0
    AL = len(approx)
    # print 'approx',approx

    new_approx = []
    for i in range(0, AL):

        x1 = approx[i % AL][0][0]
        y1 = approx[i % AL][0][1]
        x2 = approx[(i + 1) % AL][0][0]
        y2 = approx[(i + 1) % AL][0][1]
        x3 = approx[(i + 2) % AL][0][0]
        y3 = approx[(i + 2) % AL][0][1]

        l1 = np.array([x2 - x1, y2 - y1])
        l2 = np.array([x3 - x2, y3 - y2])

        dot = np.dot(l1, l2)
        angle = np.arccos(abs(dot) / (np.linalg.norm(l1) *
                                      np.linalg.norm(l2))) * 180 / np.pi
        # angle1 = np.tanh(y2 - y1, x2 - x1)
        # angle2 = np.tanh(y3 - y2, x3 - x2)

        dif0 = abs(angle - 90)
        dif1 = abs(angle - 45)
        if dif0 <= dif1:
            dif = dif0
        else:
            dif=dif1
        # print 'l1',l1
        # print 'l2',l2
        # print 'dot',dot
        # print 'norm l1',np.linalg.norm(l1)
        # print 'norm l2',np.linalg.norm(l2)
        # print 'angle',angle
        # print 'dif',dif
        # print 'dif0',dif0
        # print 'dif1',dif1
        # print 'tresh',thresh

        if dif < thresh and dif == dif0 and len(new_approx)==0 and len(approx)==3 and side_view==0:
            # print 'angle dif',angle
            # new_approx.append((approx[(i + 2) % AL]).tolist())
            # new_approx.append((approx[i % AL]).tolist())
            # new_approx.append((approx[(i + 1) % AL]).tolist())
            new_approx.append((approx[(i + 2) % AL]).tolist())
            new_approx.append((approx[i % AL]).tolist())
            new_approx.append((approx[(i + 1) % AL]).tolist())

        elif side_view == 1 and len(approx)==3 and len(new_approx)==0:
            temp = copy.deepcopy(np.array(approx)[:,0])
            temp = temp[:,0]
            temp = temp.tolist()
            x_min_index = temp.index(min(temp))
            new_approx.append((approx[(x_min_index + 1) % AL]).tolist())
            new_approx.append((approx[(x_min_index+2) % AL]).tolist())
            new_approx.append((approx[x_min_index % AL]).tolist())

        elif side_view ==2 and len(approx)==3 and len(new_approx)==0:
            temp = copy.deepcopy(np.array(approx)[:,0])
            temp = temp[:,1]
            temp = temp.tolist()
            y_min_index = temp.index(min(temp))
            new_approx.append((approx[(y_min_index + 1) % AL]).tolist())
            new_approx.append((approx[(y_min_index+2) % AL]).tolist())
            new_approx.append((approx[y_min_index % AL]).tolist())

        elif side_view ==3 and len(approx)==4 and len(new_approx)==0:
            temp = copy.deepcopy(np.array(approx)[:,0])
            temp = temp[:,0]
            temp = temp.tolist()
            x_min_index = temp.index(min(temp))
            # print 'temp',temp
            # print 'x min index',x_min_index
            new_approx.append((approx[x_min_index % AL]).tolist())
            new_approx.append((approx[(x_min_index + 1) % AL]).tolist())
            new_approx.append((approx[(x_min_index+2) % AL]).tolist())
            new_approx.append((approx[(x_min_index+3) % AL]).tolist())
            
            # print 'new approx',new_approx

        else:
            new_approx = approx

        error += dif

        if dif > thresh:
            right = False
    # print 'new approx',new_approx
    return (right, error / AL, new_approx)

def rightA(approx, thresh, view):
    right = True
    error = 0
    AL = len(approx)
    # print 'approx',approx

    new_approx = []
    for i in range(0, AL):

        x1 = approx[i % AL][0][0]
        y1 = approx[i % AL][0][1]
        x2 = approx[(i + 1) % AL][0][0]
        y2 = approx[(i + 1) % AL][0][1]
        x3 = approx[(i + 2) % AL][0][0]
        y3 = approx[(i + 2) % AL][0][1]

        l1 = np.array([x2 - x1, y2 - y1])
        l2 = np.array([x3 - x2, y3 - y2])

        dot = np.dot(l1, l2)
        angle = np.arccos(abs(dot) / (np.linalg.norm(l1) *
                                      np.linalg.norm(l2))) * 180 / np.pi
        # angle1 = np.tanh(y2 - y1, x2 - x1)
        # angle2 = np.tanh(y3 - y2, x3 - x2)

        dif0 = abs(angle - 90)
        dif1 = abs(angle - 45)
        if dif0 <= dif1:
            dif = dif0
        else:
            dif=dif1
        # print 'l1',l1
        # print 'l2',l2
        # print 'dot',dot
        # print 'norm l1',np.linalg.norm(l1)
        # print 'norm l2',np.linalg.norm(l2)
        # print 'angle',angle
        # print 'dif',dif
        # print 'dif0',dif0
        # print 'dif1',dif1
        # print 'tresh',thresh

        #parameter view is to determine the original point
        if len(approx)==3 and len(new_approx)==0 and dif<thresh:
            if view=='top' and dif==dif0:
                #watch triangle from the top, the vertex of 90 degree is the original point
                new_approx.append((approx[(i + 1) % AL]).tolist())
                new_approx.append((approx[(i + 2) % AL]).tolist())
                new_approx.append((approx[i % AL]).tolist())    
            elif view=='side_left':
                #watch triangle from the side, the leftmost point is the original point
                temp = copy.deepcopy(np.array(approx)[:,0])
                temp = temp[:,0]
                temp = temp.tolist()
                x_min_index = temp.index(min(temp))
                new_approx.append((approx[x_min_index % AL]).tolist())
                new_approx.append((approx[(x_min_index + 1) % AL]).tolist())
                new_approx.append((approx[(x_min_index+2) % AL]).tolist())
            elif view=='side_right':
                #watch triangle from the side, the rightmost point is the original point
                temp = copy.deepcopy(np.array(approx)[:,0])
                temp = temp[:,0]
                temp = temp.tolist()
                x_max_index = temp.index(max(temp))
                new_approx.append((approx[x_max_index % AL]).tolist())
                new_approx.append((approx[(x_max_index + 1) % AL]).tolist())
                new_approx.append((approx[(x_max_index+2) % AL]).tolist())  
            elif view=='side_up':
                #watch triangle from the side, the uppermost point is the original point
                temp = copy.deepcopy(np.array(approx)[:,0])
                temp = temp[:,1]
                temp = temp.tolist()
                y_min_index = temp.index(min(temp))
                new_approx.append((approx[y_min_index % AL]).tolist())
                new_approx.append((approx[(y_min_index + 1) % AL]).tolist())
                new_approx.append((approx[(y_min_index+2) % AL]).tolist())                               
            elif view=='side_down':
                #watch triangle from the side, the uppermost point is the original point
                temp = copy.deepcopy(np.array(approx)[:,0])
                temp = temp[:,1]
                temp = temp.tolist()
                y_max_index = temp.index(max(temp))
                new_approx.append((approx[y_max_index % AL]).tolist())
                new_approx.append((approx[(y_max_index + 1) % AL]).tolist())
                new_approx.append((approx[(y_max_index+2) % AL]).tolist()) 
        elif len(approx)>3 and dif<thresh:
            new_approx=approx

        error += dif

        if dif > thresh:
            right = False
    # print 'new approx',new_approx
    return (right, error / AL, new_approx)

def H_from_points(fp, tp):
    """Find homography H, such that fp is mapped to tp using the linear
       DLT method. Points are conditioned automatically.
    """
    if fp.shape != tp.shape:
        raise RuntimeError('number of points do not match')

    # Condition points
    # --from points--
    m = np.mean(fp[:2], axis=1)
    maxstd = np.max(np.std(fp[:2], axis=1)) + 1e-9
    C1 = np.diag([1 / maxstd, 1 / maxstd, 1])
    C1[0][2] = -m[0] / maxstd
    C1[1][2] = -m[1] / maxstd
    fp = np.dot(C1, fp)

    # --to points--
    m = np.mean(tp[:2], axis=1)
    maxstd = np.max(np.std(tp[:2], axis=1)) + 1e-9
    C2 = np.diag([1 / maxstd, 1 / maxstd, 1])
    C2[0][2] = -m[0] / maxstd
    C2[0][1] = -m[1] / maxstd
    tp = np.dot(C2, tp)

    # create matrix for linear method, 2 rows for each corresponding pair
    nbr_correspondences = fp.shape[1]
    A = np.zeros((2 * nbr_correspondences, 9))
    for i in range(nbr_correspondences):
        A[2 * i] = [-fp[0][i],
                    -fp[1][i],
                    -1, 0, 0, 0,
                    tp[0][i] * fp[0][i],
                    tp[0][i] * fp[1][i],
                    tp[0][i]]

        A[2 * i + 1] = [0, 0, 0,
                        -fp[0][i],
                        -fp[1][i],
                        -1,
                        tp[1][i] * fp[0][i],
                        tp[1][i] * fp[1][i],
                        tp[1][i]]

    U, S, V = np.linalg.svd(A)
    H = V[8].reshape((3, 3))

    # De-condition
    H = np.dot(np.linalg.inv(C2), np.dot(H, C1))

    # Normalize and return
    return H

def drawCircles(image, points, radius, color, thick):
    for i in range(0, len(points)):
        cv2.circle(image, tuple(map(tuple, points))[i], radius, color, thick)

    return 1


def detectColor(frame, lower, upper):
    lower = np.array(lower, dtype="uint8")
    upper = np.array(upper, dtype="uint8")

    mask = cv2.inRange(frame, lower, upper)
    green_mask = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow('hello', green_mask)

    redP = np.where(mask == 255)

    red_point = None
    if np.sum(redP) > 0:
        y = np.mean(redP[0])
        x = np.mean(redP[1])
        red_point = (int(x), int(y))
        cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), 2)

    return (red_point, mask)


def sortContour(red_point, pts_dst):

    num_dst = np.array(pts_dst)
    fir = None
    sec = None
    fir_dis = float("inf")
    sec_dis = float("inf")
    # print '########################################'
    # print 'red_point',red_point
    # print 'num_dst',num_dst

    for i in range(0, len(num_dst)):
        cur_dis = np.linalg.norm(np.array(red_point) - num_dst[i])
        if cur_dis < fir_dis:
            sec_dis = fir_dis
            sec = fir
            fir_dis = cur_dis
            fir = i
        elif cur_dis < sec_dis:
            sec_dis = cur_dis
            sec = i
        # print 'cur_dis',cur_dis
        # print 'fir_dis',fir_dis
        # print 'sec_dis',sec_dis
        # print 'fir',fir
        # print 'sec',sec

    if fir == 11 and sec == 0:
        pts_dst = pts_dst[fir:] + pts_dst[:fir]
    elif fir == 0 and sec == 11:
        fir = 11 - fir
        pts_dst = pts_dst[::-1]
        pts_dst = pts_dst[fir:] + pts_dst[:fir]
    elif fir < sec:
        pts_dst = pts_dst[fir:] + pts_dst[:fir]
    elif fir > sec:
        fir = 11 - fir
        pts_dst = pts_dst[::-1]
        pts_dst = pts_dst[fir:] + pts_dst[:fir]
    # print 'fir',fir
    # print 'pts',pts_dst
    # print '########################################'
    return pts_dst

#########################################
# This decomposes the homogrpahy matrix
# Takes two inputs
# A - intrinsic camera matrix
# H - homography between two 2D points

def decHomography(A, H):
    H = np.transpose(H)
    h1 = H[0]
    h2 = H[1]
    h3 = H[2]

    Ainv = np.linalg.inv(A)

    L = 1 / np.linalg.norm(np.dot(Ainv, h1))

    r1 = L * np.dot(Ainv, h1)
    r2 = L * np.dot(Ainv, h2)
    r3 = np.cross(r1, r2)

    T = L * np.dot(Ainv, h3)

    R = np.array([[r1], [r2], [r3]])
    R = np.reshape(R, (3, 3))
    U, S, V = np.linalg.svd(R, full_matrices=True)

    U = np.matrix(U)
    V = np.matrix(V)
    R = U * V

    return (R, T)
#
# def decHomography1(K,H):
#
#     '''
#     H is the homography matrix
#     K is the camera calibration matrix
#     T is translation
#     R is rotation
#     '''
#     H = H.T
#     h1 = H[0]
#     h2 = H[1]
#     h3 = H[2]
#     K_inv = np.linalg.inv(K)
#     L = 1 / np.linalg.norm(np.dot(K_inv, h1))
#     r1 = L * np.dot(K_inv, h1)
#     r2 = L * np.dot(K_inv, h2)
#     r3 = np.cross(r1, r2)
#     T = L * (K_inv@ h3.reshape(3, 1))
#     R = np.array([[r1], [r2], [r3]])
#     R = np.reshape(R, (3, 3))

def print2Mat(arrayNum):
    iterNum = iter(arrayNum)
    num = next(iterNum)
    matText = '[' + ' '.join(map(str, num))
    for num in arrayNum:
        matText += ';' + ' '.join(map(str, num))
    matText += ']'
    return matText


def decRotation(R):
    x = math.atan2(R[2, 1], R[2, 2])
    y = math.atan2(-R[2, 0], math.sqrt(R[2, 1] * R[2, 1] + R[2, 2] * R[2, 2]))
    z = math.atan2(R[1, 0], R[0, 0])
    return (x, y, z)

def line_intersect(x1, y1, x2, y2, x3, y3, x4, y4):
        """ returns a (x, y) tuple or None if there is no intersection """
        # (x1,y1,),(x2,y2) for L1; (x3,y3),(x4,y4) for L2
        d = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)

        if d == 0:
            return 0
        else:
            x = ((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4))/d
            y = ((x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4))/d
            return (x, y)
