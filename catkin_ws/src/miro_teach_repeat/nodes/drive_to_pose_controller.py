#!/usr/bin/python

import rospy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import tf_conversions

import math

from miro_teach_repeat.msg import Goal

MAX_V = 0.2
MAX_OMEGA = MAX_V / 0.164 # (width) = 2.439

def wrapToPi(x):
	return ((x + math.pi) % (2*math.pi)) - math.pi

class miro_drive_to_pose_controller:

	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):
		self.ready = not rospy.get_param('/wait_for_ready', False)
		self.gain_rho = rospy.get_param('~gain_distance', 0.5)
		self.gain_alpha = rospy.get_param('~gain_turn_to_point', 5.0)
		self.gain_beta = rospy.get_param('~gain_turn_to_heading', -3.0)
		self.goal_pos = None
		self.goal_theta = None
		self.stop_at_goal = None

	def setup_publishers(self):
		self.pub_cmd_vel = rospy.Publisher("/miro/control/cmd_vel", TwistStamped, queue_size=1)

	def setup_subscribers(self):
		if not self.ready:
			self.sub_ready = rospy.Subscriber("/miro/ready", Bool, self.on_ready, queue_size=1)
		self.sub_odom = rospy.Subscriber("/miro/sensors/odom/integrated", Odometry, self.process_odom_data, queue_size=1)
		self.sub_goal = rospy.Subscriber("/miro/control/goal", Goal, self.set_goal, queue_size=1)

	def on_ready(self, msg):
		if msg.data:
			self.ready = True

	def process_odom_data(self, msg):
		if self.ready:
			if self.goal_pos is None:
				return

			current_frame = tf_conversions.fromMsg(msg.pose.pose)
			theta = current_frame.M.GetRPY()[2]

			d_pos = tf_conversions.Vector(*self.goal_pos) - current_frame.p
			rho = d_pos.Norm()
			alpha = wrapToPi(math.atan2(d_pos.y(), d_pos.x()) - theta)
			beta = wrapToPi(-theta - alpha + self.goal_theta)

			v = self.gain_rho * rho
			omega = self.gain_alpha * alpha + self.gain_beta * beta

			# Scale to preserve rate of turn
			if not self.stop_at_goal or abs(v) > MAX_V or abs(omega) > MAX_OMEGA:
				turn_rate = abs(omega / v)
				turn_rate_at_max = MAX_OMEGA / MAX_V

				if turn_rate > turn_rate_at_max:
					omega = MAX_OMEGA * omega / abs(omega)
					v = MAX_OMEGA / turn_rate * v / abs(v)
				else:
					omega = MAX_V * turn_rate * omega / abs(omega)
					v = MAX_V * v / abs(v)

			motor_command = TwistStamped()
			motor_command.header.stamp = rospy.Time.now()
			motor_command.twist.linear.x = v
			motor_command.twist.angular.z = omega

			self.pub_cmd_vel.publish(motor_command)

	def set_goal(self, msg):
		self.goal_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, 0]
		self.goal_theta = tf_conversions.fromMsg(msg.pose.pose).M.GetRPY()[2]
		self.stop_at_goal = msg.stop_at_goal.data


if __name__ == "__main__":
	rospy.init_node("miro_drive_to_pose_controller")
	controller = miro_drive_to_pose_controller()
	rospy.spin()