#!/usr/bin/python

import rospy
import numpy as np
import cv2
import os
import pickle
import datetime
import time
from rospy_message_converter import message_converter
from sensor_msgs.msg import CompressedImage, JointState
import message_filters

import math
import json

import image_processing
from miro_teach_repeat.srv import ImageMatch
from miro_teach_repeat.srv import PoseOffset
class miro_localiser:

	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):	
		self.resize = image_processing.make_size(height=rospy.get_param('/image_resize_height', None), width=rospy.get_param('/image_resize_width', None))
		if self.resize[0] is None and self.resize[1] is None:
			self.resize = None

		self.image_offset_gain = rospy.get_param('~image_offset_gain', 2.3)
		
		self.last_image_msg = None
		
	def setup_publishers(self):	
		rospy.wait_for_service('/miro/match_image')
		self.match_image = rospy.ServiceProxy('/miro/match_image', ImageMatch, persistent=True)

	def setup_subscribers(self):	
		# subscribe to the images from both cameras
		self.sub_image_left = message_filters.Subscriber("/miro/sensors/cam/left/compressed", CompressedImage, queue_size=1)
		self.sub_image_right = message_filters.Subscriber("/miro/sensors/cam/right/compressed", CompressedImage, queue_size=1)
		self.sub_images = message_filters.ApproximateTimeSynchronizer((self.sub_image_left, self.sub_image_right), 5, 1.0/30.0)
		self.sub_images.registerCallback(self.process_image_data)

		self.service = rospy.Service('/miro/get_image_pose_offset', PoseOffset, self.calculate_image_pose_offset)
	
	def process_image_data(self, msg_left, msg_right):
		self.last_image_msg = (msg_left, msg_right)

	def calculate_image_pose_offset(self, srv):
		if self.last_image_msg is not None:
			image = image_processing.stitch_stereo_image_message(*self.last_image_msg)
			normalised_image = image_processing.patch_normalise_image(image, (9,9), resize=self.resize)
			image_offset = self.match_image(image_processing.image_to_msg(normalised_image))

			# positive image offset: query image is shifted left from reference image
			# this means we have done a right (negative turn) which we should correct with a positive turn
			# positive offset -> positive turn, gain = positive
			# (normalise the pixel offset by the width of the image)
			return self.image_offset_gain * float(image_offset) / normalised_image.shape[1]
		else:
			raise RuntimeError('Localiser: tried to localise before image data is received!')


if __name__ == "__main__":
	rospy.init_node("miro_localiser")
	localiser = miro_localiser()
	rospy.spin()