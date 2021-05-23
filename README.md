# motion_detector
## purpose:
Using usb_cam ROS package and cv_bridge for cv related process

## updates:
### 
1. Add transform to camera frame from center point of the feature(tag)
2. Need skimage, (pip install scikit-image) 
3. Need opencv-python 4.2 (pip install opencv-python==4.2.0.32) for python 2.7 
   To work with ROS Kinetic,you may need to config versions of opencv-python through running: 
   $sudo ln -sf /home/zach/.local/lib/python2.7/site-packages/cv2.so /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so 
## references:
https://blog.techbridge.cc/2016/11/26/ros-motion-detector/

https://github.com/RaubCamaioni/OpenCV_Position/blob/master/modular_tracking.py

https://scikit-image.org/docs/0.12.x/auto_examples/segmentation/plot_label.html
