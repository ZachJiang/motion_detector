#!/usr/bin/env python
import numpy as np
import copy
from shapely.geometry import *

def lineToFunction(line):
    "input line[[x1,y1],[x2,y2]], return k,b (ax+by+c=0)"
    # a = y2-y1, b = x1-x2, c=x2*y1-x1*y2
    a = line[1][1] - line[0][1]
    b = line[0][0] - line[1][0]
    c = line[1][0]*line[0][1] - line[0][0]*line[1][1]
    return a,b,c

def reversePoint(crease,point):
    # print 'crease',crease
    a,b,c = lineToFunction(crease)
    # print 'a bc ',a,b,c
    x = point[0]
    y = point[1]
    reversed_point = []
    if a == 0 and b != 0:
        x1 = x
        y1 = -2*c/b - y
    if b == 0 and a != 0:
        y1 = y
        x1 = -2*c/a - x
    if a !=0 and b!= 0:
        x1 = -1*(2*a*b*y + (a*a-b*b)*x + 2*a*c) / (a*a + b*b)
        y1 = -1*((b*b-a*a)*y + 2*a*b*x + 2*b*c) / (a*a + b*b)
    reversed_point.append(x1)
    reversed_point.append(y1)
    return reversed_point

def get_side_facets(crease,facet_pts):
    #get facets that on the right side and left side of the crease
    left_facets = []
    right_facets = []
    # print 'ut crease',crease
    a,b,c = lineToFunction(crease)

    for facet in facet_pts.keys():
        pts = facet_pts[facet]
        # print 'facet',facet
        for pt in pts:
            # print 'pt',pt
            product = a*pt[0]+b*pt[1]+c
            # print 'product',product
            if product<0 and abs(product)>4000:
                left_facets.append(facet)
                break
            elif product>0 and abs(product)>4000:
                right_facets.append(facet)
                break
            elif abs(product)<=4000:
                continue

    return left_facets,right_facets

def PointLineDistance(line_func,point):
    #line_func is in the form of a,b,c
    x=point[0]
    y=point[1]
    a=line_func[0]
    b=line_func[1]
    c=line_func[2]
    dis = abs(a*x+b*y+c)/np.sqrt(a*a+b*b)
    return dis

def pointsDistance(point1,point2):
    dx = point1[0]-point2[0]
    dy = point1[1]-point2[1]
    dis = np.sqrt(pow(dx,2)+pow(dy,2))
    return dis

def findFurthestPointInfo(crease_func,polygon,facets):
    info_tmp = []
    info = []
    dis_tmp = []
    for facet in facets:
        poly = polygon[facet]
        for i in range(len(poly)):
            #poly[i] is a point
            dis = PointLineDistance(crease_func,poly[i])
            info_tmp.append([dis,poly[i],facet])
            dis_tmp.append(dis)
    # print "info_tmp",info_tmp,type(info_tmp)
    # dis_tmp = np.array(info_tmp)[:,0]
    max_dis = max(dis_tmp)
    # dis_tmp = list(enumerate(dis_tmp))
    indexs = [i for i,x in enumerate(dis_tmp) if x==max_dis]
    # print "indexs",indexs
    for index in indexs:
        info.append([info_tmp[index][1],info_tmp[index][2]])
    return info

def if_point_in_list(point,points_list,dis_thresh=10):
    # because crease detection has errors, the previous method: if point in list, is not accurate
    # now we give some tolerance, if point has small errors with other points in the list, return 1
    points_list = np.array(points_list)
    points_list = points_list.tolist()
    for pt in points_list:
        dis = pointsDistance(pt,point)
        if dis <= dis_thresh:
            return 1
    return 0

def if_point_in_overlap_facets(point,points_lists,dis_thresh=10,area_thresh=100):
    #this is to determine if the point is in two overlap facets or in two adjacent facets

    #step1: determine if the point is in more than 1 facet
    lists = []
    points_lists = (np.array(points_lists)).tolist()
    for points_list in points_lists:
        is_in = if_point_in_list(point,points_list,dis_thresh)
        if is_in == 1:
            lists.append(points_list)
    # print 'point lists',points_lists
    # print 'lists',lists

    #step2: if true, determine if the two facets are adjacet or overlap
    if len(lists)>1:
        for i in range(len(lists)-1):
            line1 = LineString(lists[i])
            poly1 = Polygon(line1)
            poly1=poly1.buffer(0)
            for j in range(i+1,len(lists)):
                line2 = LineString(lists[j])
                poly2 = Polygon(line2)
                poly2=poly2.buffer(0)
                area = poly1.intersection(poly2).area
                print 'area',area
                if area>area_thresh:
                    return 1
    return 0

def frame_transform(pts,halfX,halfY,inverse=0):
    pts=np.array(pts)
    pts = pts.reshape(len(pts),2)
    new = np.ones(len(pts))
    new = new.reshape(len(pts),1)
    pts = np.append(pts,new,axis=1)
    pts = pts.T
    pts = pts.tolist()
    if inverse==0:
        # mat=np.matrix([[1,0,-1*halfX],[0,-1,halfY],[0,0,1]])
        mat=np.matrix([[0,-1,1*halfX],[-1,0,halfY],[0,0,1]])
    elif inverse==1:
        # mat=np.matrix([[1,0,1*halfX],[0,-1,halfY],[0,0,1]])
        mat=np.matrix([[0,-1,1*halfY],[-1,0,halfX],[0,0,1]])
    new_pts = np.dot(mat,pts)
    new_pts = new_pts[:2,:]
    new_pts = new_pts.T
    new_pts = new_pts.tolist()
    return new_pts

def ccw(pts):
    #this is to re-arrange pts as counter-clockwise
    pts=copy.deepcopy(pts)
    pts=np.array(pts)
    # l1=(0,1)
    l1=(0,1)
    sorted_pts=copy.deepcopy(pts)
    sorted_pts=sorted_pts.tolist()
    for i in range(len(pts)):
        pt=pts[i]
        l2=(pt[0],pt[1])
        theNorm = np.linalg.norm(l1)*np.linalg.norm(l2)
        rho = np.arcsin(np.cross(l1,l2)/theNorm)*180/np.pi
        angle=np.arccos(np.dot(l1,l2)/theNorm) * 180 / np.pi
        if rho < 0:
            angle=-angle
        angle=round(angle,2)
        sorted_pts[i].append(angle)

    pts_sorted = sorted(sorted_pts,key=(lambda x:x[2]))
    pts_sorted=np.array(pts_sorted)
    #if two angles are the same
    # print 'pts sprted',pts_sorted
    for i in range(len(pts_sorted)-1):
        angle1=pts_sorted[i][2]
        x1=copy.deepcopy(pts_sorted[i][0])
        y1=copy.deepcopy(pts_sorted[i][1])
        angle2=pts_sorted[i+1][2]
        x2=copy.deepcopy(pts_sorted[i+1][0])
        y2=copy.deepcopy(pts_sorted[i+1][1])
        if angle1==angle2:
            # print 'angle1',angle1
            temp=pts_sorted.tolist()
            if [45.0,-105.0,-156.80] in temp:
                # print '1'
                pts_sorted[i][0]=x2
                pts_sorted[i][1]=y2
                pts_sorted[i+1][0]=x1
                pts_sorted[i+1][1]=y1

    pts_sorted=pts_sorted[:,:2]
    pts_sorted=pts_sorted.tolist()

    return pts_sorted


def calcAngleofAxises(axis1,axis2):
    #reference is axis1, return angle that axis2 rotate to axis1
    #clockwise:-, counterclockwise:+
    axis1 = axis1 / np.linalg.norm(np.array(axis1))
    axis2 = axis2 / np.linalg.norm(np.array(axis2))
    #dot product
    theta = np.rad2deg(np.arccos(np.dot(axis1,axis2)))
    # print 'theta',theta
    #cross product
    # rho = np.rad2deg(np.arcsin(np.cross(axis1,axis2)))
    rho = np.cross(axis1,axis2)
    # print "rho",rho
    if rho[2]<0:
        return theta
    else:
        return -theta


def findGraspAngle(method,crease_axis,gripper_axis=[-1.0,0.0,0.0]):

    theta = calcAngleofAxises(crease_axis,gripper_axis)
    if method == "flexflip":
        angle = theta + 90
    elif method == "scoop":
        angle = theta - 90
    if angle<=-180:
        angle = angle+360
    if angle>=180:
        angle = angle-360

    return angle

def findMakeCreaseAngle(crease_axis,gripper_axis=[1.0,0.0,0.0]):
    theta = calcAngleofAxises(crease_axis,gripper_axis)
    # print 'theta',theta
    make_c_angle=theta-180
    # make_c_angle = theta + 180
    if make_c_angle>=180:
        make_c_angle=make_c_angle-360
    elif make_c_angle<=-180:
        make_c_angle=make_c_angle+360
    return make_c_angle


def pointTransformation(point,pos,rot_mat):
    # point = [float(point[0]),float(point[1]),float(0)]
    # pos = [float(pos[0]),float(pos[1]),float(pos[2])]
    pointt = [point[0],point[1],0.0]
    trans_point = np.dot(rot_mat,pointt)+pos
    trans_point = (np.array(trans_point)).tolist()
    trans_point=trans_point[0]
    return trans_point

def lineTransformation(line,pos,rot_mat):
    point1 = line[0]
    point2 = line[1]
    trans_point1 = pointTransformation(point1,pos,rot_mat)
    trans_point2 = pointTransformation(point2,pos,rot_mat)
    trans_line = [trans_point1,trans_point2]
    return trans_line

def axisTransformation(axis,rot):
    trans_axis = np.dot(rot,axis)
    trans_axis=(np.array(trans_axis)).tolist()
    trans_axis=trans_axis[0]
    return trans_axis