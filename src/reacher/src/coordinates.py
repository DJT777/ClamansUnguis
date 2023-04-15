import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool, Float32MultiArray
import numpy as np
#import tensorflow as tf
import onnxruntime as ort
from sensor_msgs.msg import RegionOfInterest
import os
import math
import time
from geometry_msgs.msg import PoseArray, Pose



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
    
 

if __name__ == '__main__':
    rospy.init_node('detection_subscriber', anonymous=True)
    ready_pub = rospy.Publisher("nn2", Bool, queue_size=10)
    ready_msg = Bool()
    ready_msg.data =  False
    ready_pub.publish(ready_msg)
    nn = NeuralNetwork()
    nav_pub = rospy.Publisher("reacher_navigation", Bool, queue_size=10)
    arm_pub = rospy.Publisher("arm_positions", Float32MultiArray, queue_size=100)
    detected = False
    while not rospy.is_shutdown():
        ready_msg = Bool()
        ready_msg.data = True
        ready_pub.publish(ready_msg)
        try:
            data = rospy.wait_for_message("detections", RegionOfInterest, timeout=.1)
            print("Message received")
            detected = True
            nav_pub.publish(Bool(False))
        except rospy.exceptions.ROSException:
            detected = False
            nav_pub.publish(Bool(True))
            continue
        if detected:
            nav_pub.publish(Bool(False))
            nav_pub.publish(Bool(False))
            x = data.x_offset
            y = data.y_offset
            width = data.width
            height = data.height
            predicted_3d_coords = nn.predict_coords([x,y])
            predicted_robotic_arm = nn.predict_reacher(predicted_3d_coords)
            print(predicted_robotic_arm)
            arm_positions = Float32MultiArray()
            arm_positions.data = predicted_robotic_arm[0].flatten().tolist()
            #print(arm_positions)
            arm_pub.publish(arm_positions)
            #dummy_arm_positions = Float32MultiArray()
            #dummy_arm_positions.data = [0, 1, 2, 3, 4]
            #arm_pub.publish(dummy_arm_positions)

