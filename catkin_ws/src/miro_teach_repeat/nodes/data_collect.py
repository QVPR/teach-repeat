#!/usr/bin/python

import rospy
import math
import time
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import tf_conversions


from miro_teach_repeat.msg import ImageAndPose
from miro_onboard.msg import CompressedImageSynchronised

DEFAULT_DISTANCE_THRESHOLD = 0.1
DEFAULT_ANGLE_THRESHOLD = 5.0

class miro_data_collect:

	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):
		self.ready = not rospy.get_param('/wait_for_ready', False)
		self.last_odom = None
		self.current_odom = None
		self.distance_threshold = rospy.get_param('~distance_threshold', DEFAULT_DISTANCE_THRESHOLD)
		self.angle_threshold = math.radians(rospy.get_param('~angle_threshold_deg', DEFAULT_ANGLE_THRESHOLD))

	def setup_publishers(self):	
		self.pub_image_pose = rospy.Publisher("/miro/image_pose", ImageAndPose, queue_size=1)

	def setup_subscribers(self):
		if not self.ready:
			self.sub_ready = rospy.Subscriber("/miro/ready", Bool, self.on_ready, queue_size=1)
		self.sub_odom = rospy.Subscriber("/miro/sensors/odom/integrated", Odometry, self.process_odom_data, queue_size=1)
		self.sub_images = rospy.Subscriber('/miro/sensors/cam/both/compressed', CompressedImageSynchronised, self.process_image_data, queue_size=1, buff_size=2**22)

	def on_ready(self, msg):
		if msg.data:
			self.ready = True

	def process_odom_data(self, msg):
		if self.ready:
			self.current_odom = msg

	def process_image_data(self, msg):
		if self.ready:
			if self.last_odom is None:
				if self.current_odom is not None:
					self.publish_image_and_pose(msg.left, msg.right, self.current_odom.pose.pose)
					self.last_odom = self.current_odom
				return

			current_frame = tf_conversions.fromMsg(self.current_odom.pose.pose)
			old_frame = tf_conversions.fromMsg(self.last_odom.pose.pose)
			difference = old_frame.Inverse() * current_frame

			delta_distance = difference.p.Norm()
			delta_theta = abs(difference.M.GetRPY()[2])

			if delta_distance > self.distance_threshold or delta_theta > self.angle_threshold:
				self.publish_image_and_pose(msg.left, msg.right, self.current_odom.pose.pose)
				self.last_odom = self.current_odom
		
	def publish_image_and_pose(self, img_left, img_right, pose):
		img_pose = ImageAndPose()
		img_pose.image_left = img_left
		img_pose.image_right = img_right
		img_pose.pose = pose
		self.pub_image_pose.publish(img_pose)


if __name__ == "__main__":
	rospy.init_node("miro_data_collect")
	collector = miro_data_collect()
	rospy.spin()