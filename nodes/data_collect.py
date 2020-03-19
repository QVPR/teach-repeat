#!/usr/bin/python

import rospy
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import tf_conversions

import math

from miro_teach_repeat.msg import ImageAndPose

DEFAULT_DISTANCE_THRESHOLD = 0.1
DEFAULT_ANGLE_THRESHOLD = 5.0

class miro_data_collect:

	def __init__(self):		
		# subscribe to integrated odometry
		self.sub_odom = rospy.Subscriber("/miro/odom_integrated", Odometry, self.process_odom_data, queue_size=1)
		# subscribe to image
		self.sub_image_left = rospy.Subscriber("/miro/sensors/caml/compressed", CompressedImage, self.process_image_data, queue_size=1)
		# publish image and pose pair
		self.pub_image_pose = rospy.Publisher("/miro/image_pose", ImageAndPose, queue_size=0)

		self.last_odom = None
		self.current_odom = None
		self.DISTANCE_THRESHOLD = rospy.get_param('~distance_threshold', DEFAULT_DISTANCE_THRESHOLD)
		self.ANGLE_THRESHOLD = math.radians(rospy.get_param('~angle_threshold_deg', DEFAULT_ANGLE_THRESHOLD))

	def process_odom_data(self, msg):
		self.current_odom = msg

	def process_image_data(self, msg):
		if self.last_odom is None:
			if self.current_odom is not None:
				self.publish_image_and_pose(msg, self.current_odom.pose.pose)
				self.last_odom = self.current_odom
			return

		current_frame = tf_conversions.fromMsg(self.current_odom.pose.pose)
		old_frame = tf_conversions.fromMsg(self.last_odom.pose.pose)
		difference = old_frame.Inverse() * current_frame

		delta_distance = difference.p.Norm()
		delta_theta = abs(difference.M.GetRPY()[2])

		if delta_distance > self.DISTANCE_THRESHOLD or delta_theta > self.ANGLE_THRESHOLD:
			self.publish_image_and_pose(msg, self.current_odom.pose.pose)
			self.last_odom = self.current_odom
		
	def publish_image_and_pose(self, img, pose):
		img_pose = ImageAndPose()
		img_pose.image = img
		img_pose.pose = pose
		self.pub_image_pose.publish(img_pose)



if __name__ == "__main__":

	rospy.init_node("miro_data_collect")
	integrator = miro_data_collect()
	rospy.spin()
