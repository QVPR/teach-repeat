#!/usr/bin/python

from enum import Enum

import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy

class miro_teleop_joy:

	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):
		self.axis_linear = rospy.get_param('~axis_linear', 1)
		self.axis_angular = rospy.get_param('~axis_angular', 2)
		self.scale_linear = rospy.get_param('~scale_linear', 0.2)
		self.scale_angular = rospy.get_param('~scale_angular', 1.0)

	def setup_publishers(self):
		self.pub_cmd_vel = rospy.Publisher("/miro/control/cmd_vel", TwistStamped, queue_size=0)

	def setup_subscribers(self):
		self.sub_odom = rospy.Subscriber("/joy", Joy, self.process_joy_data, queue_size=1)

	def process_joy_data(self, msg):
		motor_command = TwistStamped()
		motor_command.header.stamp = rospy.Time.now()
		motor_command.twist.linear.x = msg.axes[self.axis_linear] * self.scale_linear
		motor_command.twist.angular.z = msg.axes[self.axis_angular] * self.scale_angular

		self.pub_cmd_vel.publish(motor_command)

if __name__ == "__main__":
	rospy.init_node("miro_teleop_joy")
	teleop = miro_teleop_joy()
	rospy.spin()