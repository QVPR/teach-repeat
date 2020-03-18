#!/usr/bin/python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import tf_conversions

class miro_integrate_odom:

	def __init__(self):		
		# subscribe to instantaneous odometry
		self.sub_odom = rospy.Subscriber("/miro/sensors/odom", Odometry, self.process_odom_data, queue_size=1)

		# publish integrated odometry
		self.pub_odom = rospy.Publisher("/miro/odom_integrated", Odometry, queue_size=0)

		self.last_odom_time = None
		self.odom_pose = Pose()
		self.odom_pose.orientation.w = 1

	def process_odom_data(self, msg):
		if self.last_odom_time is None:
			self.last_odom_time = msg.header.stamp
			return

		delta_time = (msg.header.stamp - self.last_odom_time).to_sec()
		self.last_odom_time = msg.header.stamp

		# convert current pose and odometry to PyKDL format for calculations
		pose_frame = tf_conversions.fromMsg(self.odom_pose)
		velocity = tf_conversions.Twist(
			tf_conversions.Vector(msg.twist.twist.linear.x ,msg.twist.twist.linear.y, msg.twist.twist.linear.z),
			tf_conversions.Vector(msg.twist.twist.angular.x ,msg.twist.twist.angular.y, msg.twist.twist.angular.z)
		)

		pose_frame.Integrate(velocity, 1/delta_time)

		self.odom_pose = tf_conversions.toMsg(pose_frame)

		odom_to_publish = msg
		odom_to_publish.header.frame_id = "odom"
		odom_to_publish.pose.pose = self.odom_pose
		self.pub_odom.publish(odom_to_publish)



if __name__ == "__main__":
	rospy.init_node("miro_integrate_odom")
	integrator = miro_integrate_odom()
	rospy.spin()
