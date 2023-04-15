#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

import numpy as np
import getch
#import tensorflow as tf
from sensor_msgs.msg import RegionOfInterest
import os
import math
import time
from Arm_Lib import Arm_Device

time_1 = 500
time_2 = 1000
time_sleep = 0.5
Arm = Arm_Device()


def move_arm(input_data):
        pi = math.pi

        
        output = input_data
        
        dof_angle_limits = [(-90, 90, False),(-90, 90, False),(-90, 90, False),(-90, 90, False),(-90, 180, False),(-30, 60, True),]
        
        servo_angle_limits = [(0, 180),(0, 180),(0, 180),(0, 180),(0, 270),(0, 180),]
        
        
        servo_angles = [90] * 6
        for i, pos in enumerate(output):
            if i == 5:
              continue
        
            L, U, inversed = dof_angle_limits[i]
            A, B = servo_angle_limits[i]
            angle = np.rad2deg(float(pos))
            if not L <= angle <= U:
              angle = np.clip(angle, L, U)
            servo_angles[i] = (angle - L) * ((B-A)/(U-L)) + A
            if inversed:
              servo_angles[i] = (B-A) - (servo_angles[i] - A) + A
            if not A <= servo_angles[i] <= B:
              raise Exception("(Should Not Happen) The {}-th real world joint angle ({}) is out of range! Should be in [{}, {}]".format(i, servo_angles[i], A, B))
        Arm.Arm_serial_servo_write6(servo_angles[0], servo_angles[1], servo_angles[2], servo_angles[3], 90, 90, 1000)
        key = getch.getch()
        time.sleep(2.5)
        Arm.Arm_serial_servo_write6(servo_angles[0], 35, servo_angles[2], servo_angles[3], 90, 45, 1000)
        #key = getch.getch()
        time.sleep(2.5)
        Arm.Arm_serial_servo_write6(servo_angles[0], 35, servo_angles[2], servo_angles[3], 90, 140, 1000)
        #key = getch.getch()
        time.sleep(2.5)
        Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 140, 1000)
        #key = getch.getch()

def arm_positions_callback(msg):
    arm_positions = msg.data
    print("Received arm positions:", arm_positions)




if __name__ == '__main__':
    rospy.init_node('arm_positions_listener')
    rospy.Subscriber("arm_positions", Float32MultiArray, arm_positions_callback)
    ready_pub = rospy.Publisher("nn2", Bool, queue_size=10)
    nav_pub = rospy.Publisher("reacher_navigation", Bool, queue_size=10)
    ready_msg = Bool()
    ready_msg.data =  False
    ready_pub.publish(ready_msg)
    while(True):
     ready_pub.publish(True)
     nav_pub.publish(Bool(True))
     Arm.Arm_serial_servo_write6(90, 80, 25, 0, 90, 0, 750)
     
     try:
       data = rospy.wait_for_message("arm_positions", RegionOfInterest, timeout=.5)
       reaching = True
       move_arm(data.data)
     except rospy.exceptions.ROSException:
       print("No Arm Positions Received")
       reaching = False
       nav_pub.publish(Bool(True))
    if(reaching == True):
       print("Reaching")
    else:
       print("Not Reaching")


   
