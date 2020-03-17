#!/usr/bin/python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import tf_conversions

import cv2
import cv_bridge
import numpy as np
import math

class miro_integrate_odom:

	def __init__(self):		
		# subscribe to instantaneous odometry
		self.sub_odom = rospy.Subscriber("/miro/sensors/odom", Odometry, self.process_odom_data, queue_size=1)

		# publish integrated odometry
		self.pub_odom = rospy.Publisher("/miro/odom_integrated", Odometry, queue_size=0)

		self.last_odom_time = None
		self.odom_pose = Pose()

	def process_odom_data(self, msg):
		if self.last_odom_time is None:
			self.last_odom_time = msg.header.stamp
			return
		delta_time = (msg.header.stamp - self.last_odom_time).to_sec()
		self.last_odom_time = msg.header.stamp

		# current yaw angle
		theta = tf_conversions.transformations.euler_from_quaternion(self.quaternion_to_numpy(self.odom_pose.orientation))[2]

		# calculate velocity
		delta_x = msg.twist.twist.linear.x * delta_time * math.cos(theta)
		delta_y = msg.twist.twist.linear.x * delta_time * math.sin(theta)
		delta_theta = msg.twist.twist.angular.z * delta_time

		# update Pose
		self.odom_pose.position.x += delta_x
		self.odom_pose.position.y += delta_y
		theta += delta_theta
		self.odom_pose.orientation = tf_conversions.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, theta + delta_theta))

		# publish integrated odometry messages
		odom_to_publish = msg
		odom_to_publish.pose.pose = self.odom_pose
		self.pub_odom.publish(odom_to_publish)

	def quaternion_to_numpy(self, quaternion):
		return np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])



if __name__ == "__main__":

	rospy.init_node("miro_integrate_odom")
	integrator = miro_integrate_odom()
	rospy.spin()
