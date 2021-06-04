#!/usr/bin/env python
import numpy as np
import copy

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

def if_point_in_list(point,points_list):
    # because crease detection has errors, the previous method: if point in list, is not accurate
    # now we give some tolerance, if point has small errors with other points in the list, return 1
    for pt in points_list:
        dis = pointsDistance(pt,point)
        if dis <= 10:
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
        mat=np.matrix([[1,0,-1*halfX],[0,-1,halfY],[0,0,1]])
    elif inverse==1:
        mat=np.matrix([[1,0,1*halfX],[0,-1,halfY],[0,0,1]])
    new_pts = np.dot(mat,pts)
    new_pts = new_pts[:2,:]
    new_pts = new_pts.T
    new_pts = new_pts.tolist()
    return new_pts

def ccw(pts):
    #this is to re-arrange pts as counter-clockwise
    pts=copy.deepcopy(pts)
    pts=np.array(pts)
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
        sorted_pts[i].append(angle)

    pts_sorted = sorted(sorted_pts,key=(lambda x:x[2]))
    pts_sorted=np.array(pts_sorted)
    pts_sorted=pts_sorted[:,:2]
    pts_sorted=pts_sorted.tolist()

    return pts_sorted