#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import os
import math
import yaml
from rospy_message_converter import message_converter
from sensor_msgs.msg import Image, CameraInfo

from teach_repeat import image_processing
from teach_repeat.msg import CompressedImageSynchronised

class image_stitcher:
	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):
		self.left_cal_file = rospy.get_param('/calibration_file_left', None)
		self.right_cal_file = rospy.get_param('/calibration_file_right', None)

		if self.left_cal_file is not None:
			with open(self.left_cal_file,'r') as f:
				self.cam_left_calibration = image_processing.yaml_to_camera_info(yaml.load(f.read(), Loader=yaml.SafeLoader))
		else:
			rospy.loginfo('[stitcher] no calibration file specified for left camera. Falling back to uncalibrated image stitching.')
			self.cam_left_calibration = None
		
		if self.right_cal_file is not None:
			with open(self.right_cal_file,'r') as f:
				self.cam_right_calibration = image_processing.yaml_to_camera_info(yaml.load(f.read(), Loader=yaml.SafeLoader))
		else:
			rospy.loginfo('[stitcher] no calibration file specified for right camera. Falling back to uncalibrated image stitching.')
			self.cam_right_calibration = None

		if self.cam_left_calibration is not None and self.cam_right_calibration is not None:
			# get rectification parameters - defaults are for Miro-E with 640x480 image resolution
			self.half_field_of_view = rospy.get_param('~half_field_of_view', 60.6)
			self.half_camera_offset = rospy.get_param('~half_camera_offset', 27.0)
			self.extra_pixels_proportion = rospy.get_param('~extra_pixels_proportion', 200.0/640)
			self.blank_pixels_proportion = rospy.get_param('~blank_pixels_proportion', 200.0/640)
		else:
			# get stitching parameters without rectification - defaults are for Miro-E with 640x480 image resolution
			self.image_overlap = 1 - rospy.get_param('~half_camera_offset', 27.0)/rospy.get_param('~half_field_of_view', 60.6)

		self.first_img_seq = None
		
	def setup_publishers(self):
		self.pub_image = rospy.Publisher('sensors/cam/both', Image, queue_size=10)

	def setup_subscribers(self):
		self.sub_images = rospy.Subscriber('sensors/cam/both/compressed', CompressedImageSynchronised, self.process_image_data, queue_size=1, buff_size=2**22)

	def process_image_data(self, msg):
		n = msg.left.header.seq
		if self.first_img_seq is None:
			self.first_img_seq = n
		n -= self.first_img_seq

		if self.cam_left_calibration is not None and self.cam_right_calibration is not None:
			# todo: take fov from _ and publish it on a topic (float32array)
			full_image, _ = image_processing.rectify_stitch_stereo_image_message(msg.left, msg.right, self.cam_left_calibration, self.cam_right_calibration, compressed=True, extra_pixels_proportion=self.extra_pixels_proportion, blank_pixels_proportion=self.blank_pixels_proportion, half_field_of_view=self.half_field_of_view, half_camera_offset=self.half_camera_offset)
		else:
			full_image = image_processing.stitch_stereo_image_message(msg.left, msg.right, compressed=True, image_overlap=self.image_overlap)
		
		image_msg = image_processing.image_to_msg(full_image, 'mono8')
		image_msg.header = msg.left.header
		image_msg.header.seq = n
		self.pub_image.publish(image_msg)


if __name__ == "__main__":
	rospy.init_node("image_stitcher")
	stitcher = image_stitcher()
	rospy.spin()
