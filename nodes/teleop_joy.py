#!/usr/bin/env python

from enum import Enum

import rospy
#from geometry_msgs.msg import TwistStamped
#For Miro-B There's no TwistStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger, TriggerResponse

class teleop_joy:

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
		#self.pub_cmd_vel = rospy.Publisher("cmd_vel", TwistStamped, queue_size=1)
		self.pub_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)

	def setup_subscribers(self):
		if not self.ready:
			self.srv_ready = rospy.Service('ready_teleop', Trigger, self.on_ready)
		self.sub_odom = rospy.Subscriber("joy", Joy, self.process_joy_data, queue_size=1)

	def on_ready(self, srv):
		if not self.ready:
			self.ready = True
			return TriggerResponse(success=True)
		else:
			return TriggerResponse(success=False, message="Teleop already started.")

	def process_joy_data(self, msg):
		if self.ready:
			#motor_command = TwistStamped()
			#motor_command.header.stamp = rospy.Time.now()
			#motor_command.twist.linear.x = msg.axes[self.axis_linear] * self.scale_linear
			#motor_command.twist.angular.z = msg.axes[self.axis_angular] * self.scale_angular

			motor_command = Twist()
			motor_command.linear.x = msg.axes[self.axis_linear] * self.scale_linear
			motor_command.angular.z = msg.axes[self.axis_angular] * self.scale_angular

			self.pub_cmd_vel.publish(motor_command)

if __name__ == "__main__":
	rospy.init_node("teleop_joy")
	teleop = teleop_joy()
	rospy.spin()
