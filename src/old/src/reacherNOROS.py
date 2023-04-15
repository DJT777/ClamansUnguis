#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool
import numpy as np
#import tensorflow as tf
import onnxruntime as ort
from sensor_msgs.msg import RegionOfInterest


class NeuralNetwork:
    def __init__(self, num_classes=10):
        self.num_classes = num_classes
        self.onnx_session_coords = self._build_onnx_session_coords()
        self.coords_input_name = onnx_session_coords.get_inputs()[0].name
        self.coords_output_name = onnx_session_coords.get_outputs()[0].name


    def _build_onnx_session_coords(self):
        sess = ort.InferenceSession("coordinates2.onnx", providers=['TensorrtExecutionProvider', 'CUDAExecutionProvider', 'CPUExecutionProvider'])
        return sess

    def predict_coords(self, data):
        self.onnx_session_coords
        return sess






    def predict_robotic_arm(self, inputs):
        input_data = np.array(inputs, dtype=np.float32)
        #predicted_outputs = self.onnx_session.run(None, {'input': input_data})[0]
        return


if __name__ == '__main__':
    rospy.init_node('detection_subscriber', anonymous=True)
    nn = NeuralNetwork()
    #rospy.init_node('detection_subscriber', anonymous=True)
    nav_pub = rospy.Publisher("reacher_navigation", Bool, queue_size=10)
    detected = False
    while not rospy.is_shutdown():
        ready_pub = rospy.Publisher("nn2", Bool, queue_size=10)
        ready_msg = Bool()
        ready_msg.data = True
        ready_pub.publish(ready_msg)
        try:
            data = rospy.wait_for_message("detections", RegionOfInterest, timeout=1.0)
        except rospy.exceptions.ROSException:
            continue
        if data is not None:
            detected = True
            nav_pub.publish(Bool(False))
        else:
            detected = False
            nav_pub.publish(Bool(True))
        if detected:
            x = data.x_offset
            y = data.y_offset
            width = data.width
            height = data.height
            # Process the ROI here
            print("Detected ROI (x, y, w, h): ", x, y, width, height)
                #predicted_3d_coords = nn.predict_3d_coords(x, y, z)
                #print("Detected object ID: ", z)
                #print("3D Position (x, y, z): ", predicted_3d_coords)
                #predicted_robotic_arm = nn.predict_robotic_arm([x, y, z])
                #print("Robotic Arm Servo Positions: ", predicted_robotic_arm)
            print("")
