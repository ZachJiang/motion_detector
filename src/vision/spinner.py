#! /usr/bin/env python
import roslib

import numpy
import rospy
import tf
from std_msgs.msg import String,Empty,UInt16,Int8


# sub1 = rospy.Subscriber('cornerMatch/OpticalFlow', UInt16, callback)

started = True
last_data = 0
currentStep = 0
pub1 = rospy.Publisher('cornerMatch/startOpticalFlow', UInt16, queue_size=10)
pub2 = rospy.Publisher('steps/getStep', UInt16, queue_size = 10)

def callback1(data):
    if data>0:
        print "New message received"
    else:
        print "no opticalFlow"
    global started, last_data
    last_data = data
    if (not started):
        started = True

def callback2(data):
    # print "update step"
    global currentStep
    currentStep = data

def timer_callback1(event):
    global started, pub1, last_data, pub2, currentStep
    pub2.publish(currentStep)
    print "step",currentStep
    if (started):
        pub1.publish(last_data)


def listener():

    rospy.init_node('control', anonymous=True)

    rospy.Subscriber('cornerMatch/OpticalFlow', UInt16, callback1)
    rospy.Subscriber('steps/currentStep', UInt16,callback2)
    timer = rospy.Timer(rospy.Duration(0.01), timer_callback1)

    rospy.spin()    
    timer.shutdown()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass