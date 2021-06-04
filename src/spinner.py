#! /usr/bin/env python
import roslib

import numpy
import rospy
import tf
from std_msgs.msg import String,Empty,UInt16,Int8


# sub1 = rospy.Subscriber('cornerMatch/OpticalFlow', UInt16, callback)

started = True
last_data = 0
pub1 = rospy.Publisher('cornerMatch/startOpticalFlow', UInt16, queue_size=10)


def callback(data):
    print "New message received"
    global started, last_data
    last_data = data
    if (not started):
        started = True

def timer_callback(event):
    global started, pub1, last_data
    if (started):
        pub1.publish(last_data)


def listener():

    rospy.init_node('control', anonymous=True)

    rospy.Subscriber('cornerMatch/OpticalFlow', UInt16, callback)
    timer = rospy.Timer(rospy.Duration(0.01), timer_callback)

    rospy.spin()    
    timer.shutdown()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass