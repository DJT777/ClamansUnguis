#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import serial

detection_status = False
reacher_status = False

def detection_callback(msg):
    global detection_status
    detection_status = msg.data

def reacher_callback(msg):
    global reacher_status
    reacher_status = msg.data

if __name__ == '__main__':
    rospy.init_node('navigation_controller', anonymous=True)
    rospy.Subscriber("detection_navigation", Bool, detection_callback)
    rospy.Subscriber("reacher_navigation", Bool, reacher_callback)
    rate = rospy.Rate(10) # 10 Hz
    with serial.Serial('/dev/ttyUSB0', 9600) as ser:
        while not rospy.is_shutdown():
            if detection_status and reacher_status:
                ser.write(b'1')
            else:
                ser.write(b'0')
            rate.sleep()
