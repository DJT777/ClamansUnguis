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
from Arm_Lib import Arm_Device
import time
import getch

time_1 = 500
time_2 = 1000
time_sleep = 0.5
Arm = Arm_Device()



class NeuralNetwork:
    def __init__(self, num_classes=10):
        self.num_classes = num_classes
        self.onnx_session_coords = self._build_onnx_session_coords()
        self.onnx_session_reacher = self._build_onnx_session_reacher()
        self.coords_input_name = self.onnx_session_coords.get_inputs()[0].name
        self.coords_output_name = self.onnx_session_coords.get_outputs()[0].name
        self.reacher_input_name = self.onnx_session_reacher.get_inputs()[0].name
        self.reacher_output_name = self.onnx_session_reacher.get_outputs()[0].name



    def _build_onnx_session_coords(self):
        sess = ort.InferenceSession("coordinates2.onnx", providers=['CPUExecutionProvider'])
        return sess

    def _build_onnx_session_reacher(self):
        sess = ort.InferenceSession("MK4.onnx", providers=['CPUExecutionProvider'])
        input_shape = sess.get_inputs()[0].shape
        print("Input shape of the reacher model:", input_shape)
        return sess


    def predict_coords(self, data):
        input_data = np.array([data], dtype=np.float32)
        return self.onnx_session_coords.run([self.coords_output_name], {self.coords_input_name: input_data})

    def predict_reacher(self, data):
        input_data = np.array([data[0][0][0], data[0][0][1],0], dtype=np.float32)
        print("reacher input data: ", input_data)
        input_data = np.array([input_data], dtype=np.float32)
        return self.onnx_session_reacher.run([self.reacher_output_name], {self.reacher_input_name: input_data})
    
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

    def predict_robotic_arm(self, inputs):
        input_data = np.array(inputs, dtype=np.float32)
        #predicted_outputs = self.onnx_session.run(None, {'input': input_data})[0]
        return


if __name__ == '__main__':
    Arm.Arm_serial_servo_write6(90, 80, 25, 0, 90, 0, 750)
    rospy.init_node('detection_subscriber', anonymous=True)
    ready_pub = rospy.Publisher("nn2", Bool, queue_size=10)
    ready_msg = Bool()
    ready_msg.data =  False
    ready_pub.publish(ready_msg)
    #print("Press any key to continue...")
    #key = getch.getch()  # This will wait for a key press
    nn = NeuralNetwork()
    #rospy.init_node('detection_subscriber', anonymous=True)
    nav_pub = rospy.Publisher("reacher_navigation", Bool, queue_size=10)
    detected = False
    while not rospy.is_shutdown():
        Arm.Arm_serial_servo_write6(90, 80, 25, 0, 90, 0, 750)
        #time.sleep(5)
        #ready_pub = rospy.Publisher("nn2", Bool, queue_size=10)
        ready_msg = Bool()
        ready_msg.data = True
        ready_pub.publish(ready_msg)
        try:
            data = rospy.wait_for_message("detections", RegionOfInterest, timeout=.1)
            print("Message received")
            detected = True
            nav_pub.publish(Bool(False))
        except rospy.exceptions.ROSException:
            #print("Hello World")
            detected = False
            nav_pub.publish(Bool(True))
            continue
        #if data is not None:
        #    detected = True
        #    nav_pub.publish(Bool(False))
        #else:
        #    detected = False
        #    nav_pub.publish(Bool(True))
        if detected:
            detected = True
            nav_pub.publish(Bool(False))
            x = data.x_offset
            y = data.y_offset
            width = data.width
            height = data.height
            # Process the ROI here
            print("Detected ROI (x, y, w, h): ", x, y, width, height)
            print("Press any key to continue...")
            #key = getch.getch()  # This will wait for a key press
            print("You pressed:", key)
            predicted_3d_coords = nn.predict_coords([x,y])
            print("Press any key to continue...")
            #key = getch.getch()  # This will wait for a key press
            print("You pressed:", key)
                #print("Detected object ID: ", z)
            print("3D Position (x, y, z): ", predicted_3d_coords[0][0])
            predicted_robotic_arm = nn.predict_reacher(predicted_3d_coords)
            print("predicted arm", predicted_robotic_arm)
            print("Press any key to continue...")
            #key = getch.getch()  # This will wait for a key press
            print("You pressed:", key)
            nn.move_arm(predicted_robotic_arm, nn.onnx_session_reacher, nn.reacher_input_name, Arm)
            print("Robotic Arm Servo Positions: ", predicted_robotic_arm)
            print("Press any key to continue...")
            #key = getch.getch()  # This will wait for a key press
            print("You pressed:", key)
            print("")
