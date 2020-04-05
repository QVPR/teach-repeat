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
from std_msgs.msg import String
import message_filters

import math
import json

import image_processing
from miro_teach_repeat.srv import ImageMatch


DEFAULT_CAMERA_SETTINGS = "frame=180w@15"

class miro_localiser:

	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		# defer setting up subscribers until camera options are set to avoid receiving an empty image

	def setup_parameters(self):	
		self.resize = image_processing.make_size(height=rospy.get_param('~image_resize_height', None), width=rospy.get_param('~image_resize_width', None))
		if self.resize[0] is None and self.resize[1] is None:
			self.resize = None
		
		self.camera_settings = String(data=rospy.get_param('~camera_setup_command', DEFAULT_CAMERA_SETTINGS))

	def setup_publishers(self):	
		self.pub_camera_settings = rospy.Publisher("/miro/control/command", String, queue_size=0)
		rospy.wait_for_service('/miro/match_image')
		self.match_image = rospy.ServiceProxy('/miro/match_image', ImageMatch, persistent=True)

	def setup_subscribers(self):	
		# subscribe to the images from both cameras
		self.sub_image_left = message_filters.Subscriber("/miro/sensors/caml_stamped/compressed", CompressedImage, queue_size=1)
		self.sub_image_right = message_filters.Subscriber("/miro/sensors/camr_stamped/compressed", CompressedImage, queue_size=1)
		self.sub_images = message_filters.ApproximateTimeSynchronizer((self.sub_image_left, self.sub_image_right), 5, 1.0/30.0)
		self.sub_images.registerCallback(self.process_image_data)
	
	def process_image_data(self, msg_left, msg_right):
		image = image_processing.stitch_stereo_image(image_processing.compressed_msg_to_image(msg_left), image_processing.compressed_msg_to_image(msg_right))
		normalised_image = image_processing.patch_normalise_image(image, (9,9), resize=self.resize)
		delta_pose = self.match_image(image_processing.image_to_msg(normalised_image))
		# TODO: steer based on this pose difference

	def publish_camera_command(self):
		self.pub_camera_settings.publish(self.camera_settings)


if __name__ == "__main__":

	rospy.init_node("miro_localiser")
	localiser = miro_localiser()
	# hacky but seems we need to sleep for a bit before sending this command
	time.sleep(1)
	localiser.publish_camera_command()
	localiser.setup_subscribers()
	
	rospy.spin()

