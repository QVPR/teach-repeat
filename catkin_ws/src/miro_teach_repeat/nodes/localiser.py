#!/usr/bin/python

import rospy
import numpy as np
import cv2
import os
import pickle
import datetime
from rospy_message_converter import message_converter
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

import math
import json

import image_processing
from miro_teach_repeat.srv import ImageMatch


DEFAULT_CAMERA_SETTINGS = "frame=180w@25"

class miro_localiser:

	def __init__(self):
		# connect to the image matcher service
		rospy.wait_for_service('/miro/match_image')
		self.match_image = rospy.ServiceProxy('/miro/match_image', ImageMatch, persistent=True)
		# subscribe to image and pose pair
		self.sub_image_pose = rospy.Subscriber("/miro/sensors/caml/compressed", CompressedImage, self.process_image, queue_size=1)		

		self.resize = image_processing.make_size(height=rospy.get_param('~image_resize_height', None), width=rospy.get_param('~image_resize_width', None))
		if self.resize[0] is None and self.resize[1] is None:
			self.resize = None

		# publish camera settings
		self.pub_camera_settings = rospy.Publisher("/miro/control/command", String, queue_size=0)
		self.camera_settings = rospy.get_param('~camera_setup_command', DEFAULT_CAMERA_SETTINGS)
		self.pub_camera_settings.publish(String(data=self.camera_settings))
		
	def process_image(self, msg):
		normalised_image = image_processing.patch_normalise_msg(msg, (9,9), compressed=True, resize=self.resize)
		delta_pose = self.match_image(image_processing.image_to_msg(normalised_image))
		# TODO - steer based on this pose difference


if __name__ == "__main__":

	rospy.init_node("miro_localiser")
	integrator = miro_localiser()
	rospy.spin()
