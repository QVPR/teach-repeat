#!/usr/bin/python

# Note: Should be run onboard Miro

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion
from std_srvs.srv import Trigger
import tf.transformations

import math
import threading

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

class miro_integrate_odom:

	def __init__(self):		
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()
	
	def setup_parameters(self):
		self.last_odom_time = None
		self.reset_odom(Trigger())
		self.mutex = threading.Lock()

	def setup_publishers(self):
		self.pub_odom = rospy.Publisher("/miro/odom_integrated", Odometry, queue_size=0)

	def setup_subscribers(self):
		self.sub_odom = rospy.Subscriber("/miro/sensors/odom", Odometry, self.process_odom_data, queue_size=10)
		self.srv_reset_odom = rospy.Service("/miro/sensors/odom/reset", Trigger, self.reset_odom)

	def reset_odom(self, srv):
		self.mutex.acquire()
		self.odom_pose = Pose()
		self.odom_pose.orientation.w = 1
		self.mutex.release()

		srv.success = True
		return srv

	def process_odom_data(self, msg):
		if self.last_odom_time is None:
			self.last_odom_time = msg.header.stamp
			return

		delta_time = (msg.header.stamp - self.last_odom_time).to_sec()
		self.last_odom_time = msg.header.stamp

		if delta_time > 0.03:
			rospy.logwarn('[Odom integrator] - had a time difference of %f from the last odom message - might have dropped a message.' % delta_time)

		self.mutex.acquire() # access self.odom_pose
		pose = self.odom_pose

		# manually integrate odom because tf_conversions isn't installed on Miro
		dd = msg.twist.twist.linear.x * delta_time
		dtheta = msg.twist.twist.angular.z * delta_time

		theta = tf.transformations.euler_from_quaternion(quaternion_to_vector(pose.orientation))[2]
		dx = dd * math.cos(theta)
		dy = dd * math.sin(theta)

		self.pose.position.x += dx
		self.pose.position.y += dy
		self.pose.orientation = vector_to_quaternion(tf.transformations.quaternion_from_euler(0,0,theta+dtheta))
		self.normalise_quaternion(pose.orientation)

		self.odom_pose = pose
		self.mutex.release() # write self.odom_pose

		odom_to_publish = msg
		odom_to_publish.header.frame_id = "odom"
		odom_to_publish.pose.pose = pose
		self.pub_odom.publish(odom_to_publish)

if __name__ == "__main__":
	rospy.init_node("miro_integrate_odom")
	integrator = miro_integrate_odom()
	rospy.spin()
