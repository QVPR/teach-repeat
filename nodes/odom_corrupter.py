#!/usr/bin/python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion
import tf.transformations

import math

def normalise_quaternion(q):
		norm = math.sqrt(sum(val**2 for val in [q.x,q.y,q.z,q.w]))
		q.x /= norm
		q.y /= norm
		q.z /= norm
		q.w /= norm
		return q

def quaternion_to_vector(q):
	return [q.x,q.y,q.z,q.w]

def vector_to_quaternion(v):
	return Quaternion(*v)

def wrapToPi(x):
	return ((x + math.pi) % (2*math.pi)) - math.pi

class corrupt_odom:
	def __init__(self):		
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()
	
	def setup_parameters(self):
		self.last_odom_message = None
		self.pose = Pose()
		self.pose.orientation.w = 1

		self.TRANSLATION_ERROR_FACTOR = rospy.get_param('~translation_error_factor', 1.0)
		self.ROTATION_ERROR_FACTOR = rospy.get_param('~rotation_error_factor', 1.0)

	def setup_publishers(self):
		self.pub_odom = rospy.Publisher("odom_corrupted", Odometry, queue_size=0)

	def setup_subscribers(self):
		self.sub_odom = rospy.Subscriber("odom", Odometry, self.process_odom_data, queue_size=10)

	def process_odom_data(self, msg):
		if self.last_odom_message is None:
			self.last_odom_message = msg
			return

		movement_x = msg.pose.pose.position.x - self.last_odom_message.pose.pose.position.x
		movement_y = msg.pose.pose.position.y - self.last_odom_message.pose.pose.position.y
		movement_direction = math.atan2(movement_y, movement_x)
		movement_distance = math.sqrt(movement_x*movement_x + movement_y*movement_y)
		last_odom_theta = tf.transformations.euler_from_quaternion(quaternion_to_vector(self.last_odom_message.pose.pose.orientation))[2]
		current_odom_theta = tf.transformations.euler_from_quaternion(quaternion_to_vector(msg.pose.pose.orientation))[2]
		turn_before_movement = wrapToPi(movement_direction - last_odom_theta)
		turn_after_movement = wrapToPi(current_odom_theta - movement_direction)

		# to repeat movement, do turn_before_movement, drive movement_distance, do turn_after_movement
		pose_theta = tf.transformations.euler_from_quaternion(quaternion_to_vector(pose.orientation))[2]
		pose_theta += turn_before_movement * self.ROTATION_ERROR_FACTOR

		movement_distance *= self.TRANSLATION_ERROR_FACTOR
		pose.position.x += movement_distance * math.cos(pose_theta)
		pose.position.y += movement_distance * math.sin(pose_theta)

		pose_theta += turn_after_movement * self.ROTATION_ERROR_FACTOR
		pose.orientation = vector_to_quaternion(tf.transformations.quaternion_from_euler(0,0,pose_theta))
		normalise_quaternion(pose.orientation)

		odom_to_publish = msg
		odom_to_publish.header.frame_id = "odom_corrupted"
		odom_to_publish.pose.pose = pose
		self.pub_odom.publish(odom_to_publish)

		self.last_odom_message = msg

if __name__ == "__main__":
	rospy.init_node("corrupt_odom")
	corrupter = corrupt_odom()
	rospy.spin()
