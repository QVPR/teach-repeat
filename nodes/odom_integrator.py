#!/usr/bin/python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import tf_conversions

import math

class miro_integrate_odom:

	def __init__(self):		
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()
	
	def setup_parameters(self):
		self.last_odom_time = None
		self.odom_pose = Pose()
		self.odom_pose.orientation.w = 1
		self.dummy_theta = 0

	def setup_publishers(self):
		self.pub_odom = rospy.Publisher("/miro/odom_integrated", Odometry, queue_size=0)

	def setup_subscribers(self):
		self.sub_odom = rospy.Subscriber("/miro/sensors/odom", Odometry, self.process_odom_data, queue_size=10)

	def process_odom_data(self, msg):
		if self.last_odom_time is None:
			self.last_odom_time = msg.header.stamp
			return

		delta_time = (msg.header.stamp - self.last_odom_time).to_sec()
		# print('this time = ' + str(msg.header.stamp))
		# print('last time = ' + str(self.last_odom_time))
		# print('delta time = ' + str(delta_time) + ' -> ' + str(1/delta_time))
		self.last_odom_time = msg.header.stamp

		if delta_time > 0.03:
			rospy.logwarn('[Odom integrator] - had a time difference of %f from the last odom message - might have dropped a message.' % delta_time)


		# convert current pose and odometry to PyKDL format for calculations
		pose_frame = tf_conversions.fromMsg(self.odom_pose)
		velocity = tf_conversions.Twist(
			tf_conversions.Vector(msg.twist.twist.linear.x ,msg.twist.twist.linear.y, msg.twist.twist.linear.z),
			tf_conversions.Vector(msg.twist.twist.angular.x ,msg.twist.twist.angular.y, msg.twist.twist.angular.z)
		)

		# print(str(math.degrees(pose_frame.M.GetRPY()[2])) + ' [' + str(delta_time) + ']')

		pose_frame.Integrate(velocity, 1/delta_time)
		# pose_frame.Integrate(velocity, 50.0) # Miro always scales by 1/50 Hz - not by real delta time
		self.dummy_theta += round(msg.twist.twist.angular.z / 0.541171848774)

		# print('%f, (%f, %f)' % (self.dummy_theta, msg.twist.twist.angular.z, round(msg.twist.twist.angular.z / 0.541171848774)))

		# print('delta t vs. 50 Hz: %f - %f' % (math.degrees(pose_frame.M.GetRPY()[2]),math.degrees(self.dummy_theta)))

		self.odom_pose = tf_conversions.toMsg(pose_frame)
		self.normalise_quaternion(self.odom_pose.orientation)

		odom_to_publish = msg
		odom_to_publish.header.frame_id = "odom"
		odom_to_publish.pose.pose = self.odom_pose
		self.pub_odom.publish(odom_to_publish)
	
	def normalise_quaternion(self, q):
		norm = math.sqrt(sum(val**2 for val in [q.x,q.y,q.z,q.w]))
		q.x /= norm
		q.y /= norm
		q.z /= norm
		q.w /= norm
		return q

if __name__ == "__main__":
	rospy.init_node("miro_integrate_odom")
	integrator = miro_integrate_odom()
	rospy.spin()
