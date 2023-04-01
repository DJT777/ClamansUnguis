#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool
import numpy as np
import tensorflow as tf
import onnxruntime as ort


class NeuralNetwork:
    def __init__(self, num_classes=10):
        self.num_classes = num_classes
        self.model = self._build_model()
        #self.onnx_session = self._build_onnx_session()


    def _build_model(self):
        model = tf.keras.models.Sequential([
            tf.keras.layers.Dense(64, activation='relu', input_shape=(2 + self.num_classes,)),
            tf.keras.layers.Dense(64, activation='relu'),
            tf.keras.layers.Dense(3, activation=None)
        ])
        model.compile(optimizer='adam', loss='mse')
        return model

    def _build_onnx_session(self):
        sess = ort.InferenceSession("robotic_arm.onnx")
        return sess

    def preprocess_data(self, x, y, z):
        x_scaled = x / 100.0
        y_scaled = y / 100.0
        z_encoded = np.zeros(self.num_classes)
        z_encoded[z] = 1
        return np.concatenate([x_scaled, y_scaled, z_encoded])

    def predict_3d_coords(self, x, y, z):
        input_data = self.preprocess_data(x, y, z)
        predicted_coords = self.model.predict(np.array([input_data]))[0]
        return predicted_coords

    def predict_robotic_arm(self, inputs):
        input_data = np.array(inputs, dtype=np.float32)
        #predicted_outputs = self.onnx_session.run(None, {'input': input_data})[0]
        #return predicted_outputs


if __name__ == '__main__':
    rospy.init_node('detection_subscriber', anonymous=True)
    nn = NeuralNetwork()
    #rospy.init_node('detection_subscriber', anonymous=True)
    pub = rospy.Publisher("reacher_navigation", Bool, queue_size=10)
    detected = False
    while not rospy.is_shutdown():
        ready_pub = rospy.Publisher("nn2", Bool, queue_size=10)
        ready_msg = Bool()
        ready_msg.data = True
        ready_pub.publish(ready_msg)
        try:
            data = rospy.wait_for_message("detections", PoseArray, timeout=1.0)
        except rospy.exceptions.ROSException:
            continue
        if data.poses:
            detected = True
            pub.publish(Bool(False))
        else:
            detected = False
            pub.publish(Bool(True))
        if detected:
            for pose in data.poses:
                x = pose.position.x
                y = pose.position.y
                z = int(round(pose.position.z))
                predicted_3d_coords = nn.predict_3d_coords(x, y, z)
                print("Detected object ID: ", z)
                print("3D Position (x, y, z): ", predicted_3d_coords)
                predicted_robotic_arm = nn.predict_robotic_arm([x, y, z])
                print("Robotic Arm Servo Positions: ", predicted_robotic_arm)
                print("")