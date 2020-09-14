#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import tf_conversions

class drive_straight_controller:

	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):
		self.gain_distance = rospy.get_param('~gain_distance', 5.0)
		self.gain_turn = rospy.get_param('~gain_turn', 2.0)

	def setup_publishers(self):
		self.pub_cmd_vel = rospy.Publisher("cmd_vel", TwistStamped, queue_size=0)

	def setup_subscribers(self):
		self.sub_odom = rospy.Subscriber("odom", Odometry, self.process_odom_data, queue_size=1)

	def process_odom_data(self, msg):
		current_frame = tf_conversions.fromMsg(msg.pose.pose)
		d = current_frame.p.y()
		theta = current_frame.M.GetRPY()[2]

		turn_command = -self.gain_distance * d - self.gain_turn * theta

		motor_command = TwistStamped()
		motor_command.header.stamp = rospy.Time.now()
		motor_command.twist.linear.x = 0.1
		motor_command.twist.angular.z = turn_command

		self.pub_cmd_vel.publish(motor_command)

if __name__ == "__main__":
	rospy.init_node("drive_straight_controller")
	controller = drive_straight_controller()
	rospy.spin()
