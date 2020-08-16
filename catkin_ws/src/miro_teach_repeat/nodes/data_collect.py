#!/usr/bin/python

import rospy
import math
import time
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger, TriggerResponse
import tf_conversions

from miro_teach_repeat.srv import SaveImageAndPose, SaveImageAndPoseRequest

DEFAULT_DISTANCE_THRESHOLD = 0.1
DEFAULT_ANGLE_THRESHOLD = 5.0

class data_collect:

	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):
		self.ready = not rospy.get_param('/wait_for_ready', False)
		self.last_odom = None
		self.current_odom = None
		self.zero_odom_offset = None
		self.distance_threshold = rospy.get_param('~distance_threshold', DEFAULT_DISTANCE_THRESHOLD)
		self.angle_threshold = math.radians(rospy.get_param('~angle_threshold_deg', DEFAULT_ANGLE_THRESHOLD))

	def setup_publishers(self):
		rospy.wait_for_service('save_image_pose')
		self.save_image_and_pose = rospy.ServiceProxy('save_image_pose', SaveImageAndPose, persistent=True)

	def setup_subscribers(self):
		if not self.ready:
			self.srv_ready = rospy.Service('ready_data_collect', Trigger, self.on_ready)
		self.sub_odom = rospy.Subscriber("odom", Odometry, self.process_odom_data, queue_size=1)
		self.sub_images = rospy.Subscriber('image', Image, self.process_image_data, queue_size=1, buff_size=2**22)

	def on_ready(self, srv):
		if not self.ready:
			self.ready = True
			return TriggerResponse(success=True)
		else:
			return TriggerResponse(success=False, message="Data collect already started.")

	def process_odom_data(self, msg):
		if self.ready:
			if self.current_odom is None:
				self.zero_odom_offset = tf_conversions.fromMsg(msg.pose.pose)
			self.current_odom = self.subtract_odom(msg, self.zero_odom_offset)

	def subtract_odom(self, odom, odom_frame_to_subtract):
		odom_frame = tf_conversions.fromMsg(odom.pose.pose) 
		subtracted_odom = odom_frame_to_subtract.Inverse() * odom_frame
		odom.pose.pose = tf_conversions.toMsg(subtracted_odom)
		return odom

	def process_image_data(self, msg):
		if self.ready:
			if self.last_odom is None:
				if self.current_odom is not None:
					self.save_data(msg)
				return

			current_frame = tf_conversions.fromMsg(self.current_odom.pose.pose)
			old_frame = tf_conversions.fromMsg(self.last_odom.pose.pose)
			difference = old_frame.Inverse() * current_frame

			delta_distance = difference.p.Norm()
			delta_theta = abs(difference.M.GetRPY()[2])

			if delta_distance > self.distance_threshold or delta_theta > self.angle_threshold:
				self.save_data(msg)

	def save_data(self, img):
		response = self.save_image_and_pose(SaveImageAndPoseRequest(img, self.current_odom.pose.pose))
		self.last_odom = self.current_odom
		if not response.success:
			rospy.logerr('Data Collection - couldn\'t save data. Err: %s' % response.message)


if __name__ == "__main__":
	rospy.init_node("data_collect")
	collector = data_collect()
	rospy.spin()