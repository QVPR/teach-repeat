#!/usr/bin/python

import rospy
from geometry_msgs.msg import TwistStamped, Twist

class twist_stamper:

	def __init__(self):
		# subscribe to teleop commands
		self.sub_cmd_vel = rospy.Subscriber("cmd_vel_unstamped", Twist, self.process_twist, queue_size=1)
		# publish motor commands
		self.pub_cmd_vel = rospy.Publisher("cmd_vel", TwistStamped, queue_size=0)

	def process_twist(self, msg):
		msg_stamped = TwistStamped()
		msg_stamped.header.stamp = rospy.Time.now()
		msg_stamped.twist = msg
		self.pub_cmd_vel.publish(msg_stamped)

if __name__ == "__main__":
	# intialise this program as a ROS node
	rospy.init_node("twist_stamper")
	# start the demo program
	demo = twist_stamper()
	# spin makes the program run while ROS is running
	rospy.spin()