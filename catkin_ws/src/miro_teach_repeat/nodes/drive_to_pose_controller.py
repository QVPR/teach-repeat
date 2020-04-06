#!/usr/bin/python

import rospy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import tf_conversions

import math

MAX_V = 0.4
MAX_OMEGA = MAX_V / 0.164 # 2.439

def wrapToPi(x):
	return ((x + math.pi) % (2*math.pi)) - math.pi

class miro_drive_to_pose_controller:

	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):
		self.gain_rho = rospy.get_param('~gain_distance', 0.3)
		self.gain_alpha = rospy.get_param('~gain_turn_to_point', 5.0)
		self.gain_beta = rospy.get_param('~gain_turn_to_heading', -3.0)
		self.target = (2.4, -1.8, 0)
		self.target_theta = -math.pi/2

	def setup_publishers(self):
		self.pub_cmd_vel = rospy.Publisher("/miro/control/cmd_vel", TwistStamped, queue_size=0)

	def setup_subscribers(self):
		self.sub_odom = rospy.Subscriber("/miro/sensors/odom/integrated", Odometry, self.process_odom_data, queue_size=1)

	def process_odom_data(self, msg):
		current_frame = tf_conversions.fromMsg(msg.pose.pose)
		theta = current_frame.M.GetRPY()[2]

		print(str(current_frame.p) + ', ' + str(theta))

		d_pos = tf_conversions.Vector(*self.target) - current_frame.p
		
		rho = d_pos.Norm()
		alpha = wrapToPi(math.atan2(d_pos.y(), d_pos.x()) - theta)
		beta = wrapToPi(-theta - alpha + self.target_theta)

		v = self.gain_rho * rho
		omega = self.gain_alpha * alpha + self.gain_beta * beta

		# Scale to preserve rate of turn #and not slow down
		if abs(v) > MAX_V or abs(omega) > MAX_OMEGA:
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

		if rho < 0.15:
			# self.target = (0,1.0,0)
			# self.target_theta = math.pi
			return

		self.pub_cmd_vel.publish(motor_command)

if __name__ == "__main__":
	rospy.init_node("miro_drive_to_pose_controller")
	controller = miro_drive_to_pose_controller()
	rospy.spin()