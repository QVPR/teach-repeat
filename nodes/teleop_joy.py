#!/usr/bin/python

from enum import Enum

import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

class miro_teleop_joy:

	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):
		self.ready = not rospy.get_param('/wait_for_ready', False)
		self.axis_linear = rospy.get_param('~axis_linear', 1)
		self.axis_angular = rospy.get_param('~axis_angular', 2)
		self.scale_linear = rospy.get_param('~scale_linear', 0.2)
		self.scale_angular = rospy.get_param('~scale_angular', 1.0)

	def setup_publishers(self):
		self.pub_cmd_vel = rospy.Publisher("/miro/control/cmd_vel", TwistStamped, queue_size=1)

	def setup_subscribers(self):
		if not self.ready:
			self.sub_ready = rospy.Subscriber("/miro/ready", Bool, self.on_ready, queue_size=1)
		self.sub_odom = rospy.Subscriber("/joy", Joy, self.process_joy_data, queue_size=1)

	def on_ready(self, msg):
		if msg.data:
			self.ready = True
	
	def process_joy_data(self, msg):
		if self.ready:
			motor_command = TwistStamped()
			motor_command.header.stamp = rospy.Time.now()
			motor_command.twist.linear.x = msg.axes[self.axis_linear] * self.scale_linear
			motor_command.twist.angular.z = msg.axes[self.axis_angular] * self.scale_angular

			self.pub_cmd_vel.publish(motor_command)

if __name__ == "__main__":
	rospy.init_node("miro_teleop_joy")
	teleop = miro_teleop_joy()
	rospy.spin()