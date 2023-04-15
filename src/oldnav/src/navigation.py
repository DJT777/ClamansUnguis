#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
#import serial
from sensor_msgs.msg import RegionOfInterest

nn1_ready = False
nn2_ready = False
detection_status = False
reacher_status = False

def detection_callback(msg):
    global detection_status
    detection_status = msg.data

def reacher_callback(msg):
    global reacher_status
    reacher_status = msg.data



def nn1_callback(msg):
    global nn1_ready
    nn1_ready = msg.data

def nn2_callback(msg):
    global nn2_ready
    nn2_ready = msg.data


if __name__ == '__main__':
    rospy.init_node('navigation_controller', anonymous=True)
    rospy.Subscriber("vision_navigation", Bool, detection_callback)
    rospy.Subscriber("reacher_navigation", Bool, reacher_callback)
    nn1_sub = rospy.Subscriber("nn1", Bool, nn1_callback)
    nn2_sub = rospy.Subscriber("nn2", Bool, nn2_callback)
    while(True):
        try:
            nn1_msg = rospy.wait_for_message("nn1", Bool, timeout=1.0)
            nn2_msg = rospy.wait_for_message("nn2", Bool, timeout=1.0)
        except rospy.exceptions.ROSException:
            print("Timed out waiting for messages")
            continue
        if(nn1_ready and nn2_ready):
          while(True):
            
            if(reacher_status == False):
              print("Navigation Halted")
            elif(reacher_status == True):
              print("Navigation resumed")
            # rate = rospy.Rate(10) # 10 Hz
            #  with serial.Serial('/dev/ttyUSB0', 9600) as ser:
            #     while not rospy.is_shutdown():
            #
            #         if detection_status and reacher_status:
            #             ser.write(b'1')
            #         else:
            #             ser.write(b'0')
            #         rate.sleep()
        else:
          print("waiting")
