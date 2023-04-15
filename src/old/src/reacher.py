#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool
import numpy as np
#import tensorflow as tf
import onnxruntime as ort
from sensor_msgs.msg import RegionOfInterest
import os
import math
import time
import getch




def move_arm(self, input_data, sess, input_name, Arm):
        pi = math.pi

        
        output = input_data
        
        dof_angle_limits = [(-90, 90, False),(-90, 90, False),(-90, 90, False),(-90, 90, False),(-90, 180, False),(-30, 60, True),]
        
        servo_angle_limits = [(0, 180),(0, 180),(0, 180),(0, 180),(0, 270),(0, 180),]
        
        
        servo_angles = [90] * 6
        for i, pos in enumerate(output[0][0]):
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
	#time.sleep(5)
        Arm.Arm_serial_servo_write6(servo_angles[0], 35, servo_angles[2], servo_angles[3], 90, 45, 1000)
        #key = getch.getch()
	#time.sleep(5)
        Arm.Arm_serial_servo_write6(servo_angles[0], 35, servo_angles[2], servo_angles[3], 90, 140, 1000)
        #key = getch.getch()
	#time.sleep(5)
        Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 140, 1000)
        #key = getch.getch()

def arm_positions_callback(msg):
    arm_positions = msg.data
    print("Received arm positions:", arm_positions)




if __name__ == '__main__':
    rospy.init_node('arm_positions_listener', anonymous=True)
    rospy.Subscriber("arm_positions", Float32MultiArray, arm_positions_callback)
    rospy.spin()
    try:
     data = rospy.wait_for_message("arm_positions", RegionOfInterest, timeout=.1)
     print("Message received")
     detected = True
     nav_pub.publish(Bool(False))
    except rospy.exceptions.ROSException:
     print("No Arm Positions Received")
     detected = False
     nav_pub.publish(Bool(True))
     continue


   
