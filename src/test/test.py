import rospy
from std_msgs.msg import Bool

rospy.init_node('test')

pub = rospy.Publisher("reacher_navigation", Bool, queue_size=10)
ready_msg = Bool()
ready_msg.data = True
while(True):
	pub.publish(True)
	print("Hello World")
