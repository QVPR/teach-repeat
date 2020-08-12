#!/usr/bin/python

import rospy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import tf_conversions

import math

from miro_teach_repeat.msg import Goal


def wrapToPi(x):
	return ((x + math.pi) % (2*math.pi)) - math.pi

def rho_alpha_beta(dx, dy, theta, goal_theta):
	rho = math.sqrt(dx*dx + dy*dy)
	alpha = wrapToPi(math.atan2(dy, dx) - theta)
	beta = wrapToPi(-theta - alpha + goal_theta)
	return rho, alpha, beta

def check_control_parameters(gain_rho, gain_alpha, gain_beta):
	if gain_rho <= 0:
		print('Drive to pose controller: Control unstable! gain_rho must be > 0')
	if gain_beta >= 0:
		print('Drive to pose controller: Control unstable! gain_beta must be < 0')
	if gain_alpha - gain_rho <= 0:
		print('Drive to pose controller: Control unstable! gain_alpha must be > gain_rho')


class drive_to_pose_controller:

	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):
		self.ready = not rospy.get_param('/wait_for_ready', False)
		self.gain_rho = rospy.get_param('~gain_distance', 0.5)
		self.gain_alpha = rospy.get_param('~gain_turn_to_point', 5.0)
		self.gain_beta = rospy.get_param('~gain_turn_to_heading', -3.0)
		self.gain_theta = rospy.get_param('~gain_turn_on_spot', 7.0)
		check_control_parameters(self.gain_rho, self.gain_alpha, self.gain_beta)
		self.max_v = rospy.get_param('~max_v', 0.2)
		self.min_omega = rospy.get_param('~min_omega', 0.15)
		self.max_omega = rospy.get_param('~max_omega', 0.93)
		self.goal_pos = None
		self.goal_theta = None
		self.stop_at_goal = None
		self.turning_goal_distance = rospy.get_param('/goal_pose_seperation', 0.2) * rospy.get_param('/turning_target_range_distance_ratio', 0.5)

	def setup_publishers(self):
		self.pub_cmd_vel = rospy.Publisher("cmd_vel", TwistStamped, queue_size=1)

	def setup_subscribers(self):
		if not self.ready:
			self.sub_ready = rospy.Subscriber("ready", Bool, self.on_ready, queue_size=1)
		self.sub_odom = rospy.Subscriber("odom", Odometry, self.process_odom_data, queue_size=1)
		self.sub_goal = rospy.Subscriber("goal", Goal, self.set_goal, queue_size=1)

	def scale_velocities(self, v, omega, stop_at_goal):
		if not stop_at_goal or abs(v) > self.max_v or abs(omega) > self.max_omega:
			# Scale to preserve rate of turn
			if omega == 0:
				v = self.max_v
				omega = 0
			elif v == 0:
				v = 0
				omega = math.copysign(self.min_omega, omega)
			else:
				turn_rate = abs(omega / v)
				turn_rate_at_max = self.max_omega / self.max_v

				if turn_rate > turn_rate_at_max:
					omega = math.copysign(self.max_omega, omega)
					v = self.max_omega / turn_rate
				else:
					omega = math.copysign(self.max_v * turn_rate, omega)
					v = self.max_v
		elif abs(omega) < self.min_omega and v == 0:
			omega = math.copysign(self.min_omega, omega)
		return v, omega

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
			rho, alpha, beta = rho_alpha_beta(d_pos.x(), d_pos.y(), theta, self.goal_theta)

			if rho < self.turning_goal_distance or alpha > math.pi/2 and alpha < -math.pi/2:
				# we're at a turning goal, so we should just turn
				# or we're not in the stable region of the controller - rotate to face the goal
				# 	better would be to reverse the direction of motion, but this would break along-path localisation
				v = 0
				omega = self.gain_theta * wrapToPi(self.goal_theta-theta)
			else:
				# we're in the stable region of the controller
				v = self.gain_rho * rho
				omega = self.gain_alpha * alpha + self.gain_beta * beta

			v, omega = self.scale_velocities(v, omega, self.stop_at_goal)

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
	rospy.init_node("drive_to_pose_controller")
	controller = drive_to_pose_controller()
	rospy.spin()
