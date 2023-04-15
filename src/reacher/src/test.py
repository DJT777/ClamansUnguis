import rospy
from std_msgs.msg import Float32MultiArray

rospy.init_node('test_publisher', anonymous=True)
arm_pub = rospy.Publisher("arm_positions", Float32MultiArray, queue_size=100)

rate = rospy.Rate(1)  # 1 Hz

while not rospy.is_shutdown():
    print("Hello World")
    arm_positions = Float32MultiArray()
    arm_positions.data = [0, 1, 2, 3, 4]
    arm_pub.publish(arm_positions)
    rate.sleep()  # Add sleep to maintain the loop rate

