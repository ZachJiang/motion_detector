#!/usr/bin/env python

import cv2
import numpy as np
import glob
 
img_array = []
file_list = []
size = None

for filename in glob.glob('./diag_fold_sample2/*.jpg'):
    file_list.append(filename)

file_list.sort()

for i in file_list:
    frame = cv2.imread(i)
    height, width, layers = frame.shape
    size = (width,height)
    img_array.append(frame)

 
 
out = cv2.VideoWriter('project002.avi',cv2.VideoWriter_fourcc(*'DIVX'), 15, size)
 
for i in range(len(img_array)):
    out.write(img_array[i])
out.release()