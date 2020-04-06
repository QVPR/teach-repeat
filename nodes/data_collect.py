#!/usr/bin/python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import tf_conversions
import message_filters

import math
import time

from miro_teach_repeat.msg import ImageAndPose

DEFAULT_DISTANCE_THRESHOLD = 0.1
DEFAULT_ANGLE_THRESHOLD = 5.0
DEFAULT_CAMERA_SETTINGS = "frame=180w@25"

class miro_data_collect:

	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		# defer setting up subscribers until camera options are set to avoid receiving an empty image

	def setup_parameters(self):	
		self.last_odom = None
		self.current_odom = None
		self.DISTANCE_THRESHOLD = rospy.get_param('~distance_threshold', DEFAULT_DISTANCE_THRESHOLD)
		self.ANGLE_THRESHOLD = math.radians(rospy.get_param('~angle_threshold_deg', DEFAULT_ANGLE_THRESHOLD))

		self.camera_settings = String(data=rospy.get_param('~camera_setup_command', DEFAULT_CAMERA_SETTINGS))

	def setup_publishers(self):	
		self.pub_camera_settings = rospy.Publisher("/miro/control/command", String, queue_size=0)
		self.pub_image_pose = rospy.Publisher("/miro/image_pose", ImageAndPose, queue_size=0)

	def setup_subscribers(self):
		self.sub_odom = rospy.Subscriber("/miro/odom/integrated", Odometry, self.process_odom_data, queue_size=1)

		# subscribe to the images from both cameras
		self.sub_image_left = message_filters.Subscriber("/miro/sensors/cam/left/compressed", CompressedImage, queue_size=1)
		self.sub_image_right = message_filters.Subscriber("/miro/sensors/cam/right/compressed", CompressedImage, queue_size=1)
		self.sub_images = message_filters.ApproximateTimeSynchronizer((self.sub_image_left, self.sub_image_right), 5, 1.0/30.0)
		self.sub_images.registerCallback(self.process_image_data)
		
	def process_odom_data(self, msg):
		self.current_odom = msg

	def process_image_data(self, msg_left, msg_right):
		if self.last_odom is None:
			if self.current_odom is not None:
				self.publish_image_and_pose(msg_left, msg_right, self.current_odom.pose.pose)
				self.last_odom = self.current_odom
			return

		current_frame = tf_conversions.fromMsg(self.current_odom.pose.pose)
		old_frame = tf_conversions.fromMsg(self.last_odom.pose.pose)
		difference = old_frame.Inverse() * current_frame

		delta_distance = difference.p.Norm()
		delta_theta = abs(difference.M.GetRPY()[2])

		if delta_distance > self.DISTANCE_THRESHOLD or delta_theta > self.ANGLE_THRESHOLD:
			self.publish_image_and_pose(msg_left, msg_right, self.current_odom.pose.pose)
			self.last_odom = self.current_odom
		
	def publish_image_and_pose(self, img_left, img_right, pose):
		img_pose = ImageAndPose()
		img_pose.image_left = img_left
		img_pose.image_right = img_right
		img_pose.pose = pose
		self.pub_image_pose.publish(img_pose)

	def publish_camera_command(self):
		self.pub_camera_settings.publish(self.camera_settings)



if __name__ == "__main__":

	rospy.init_node("miro_data_collect")
	collector = miro_data_collect()
	# hacky but seems we need to sleep for a bit before sending this command
	time.sleep(1)
	# collector.publish_camera_command()
	collector.setup_subscribers()
	
	rospy.spin()

