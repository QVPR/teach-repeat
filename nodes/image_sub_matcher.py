#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import os
import yaml
import json
import datetime
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32MultiArray, Float32MultiArray, MultiArrayDimension

import teach_repeat.image_processing as image_processing
from teach_repeat.srv import ImageMatch, ImageMatchResponse
from sensor_msgs.msg import Image

# Do not need for the Multi-robot system
#def get_image_files_from_dir(file_dir, file_ending):
#	files = [f for f in os.listdir(file_dir) if f.endswith(file_ending)]
#	files.sort()
#	return [file_dir+f for f in files]

class image_matcher:

	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):
		self.pub_image_match_debug = rospy.Publisher('match_image_debug', Image, queue_size=0)

		self.left_cal_file = rospy.get_param('/calibration_file_left', None)
		self.right_cal_file = rospy.get_param('/calibration_file_right', None)

		# For Multi-Robot Student Delay
		self.frames_delay = rospy.get_param('~frames_delay', 10)

		if self.left_cal_file is not None:
			with open(self.left_cal_file,'r') as f:
				self.cam_left_calibration = image_processing.yaml_to_camera_info(yaml.load(f.read(), Loader=yaml.SafeLoader))
		else:
			rospy.loginfo('[Image Matcher] no calibration file for left camera specified. Assuming not calibrated')
			self.cam_left_calibration = CameraInfo()

		if self.right_cal_file is not None:
			with open(self.right_cal_file,'r') as f:
				self.cam_right_calibration = image_processing.yaml_to_camera_info(yaml.load(f.read(), Loader=yaml.SafeLoader))
		else:
			rospy.loginfo('[Image Matcher] no calibration file for right camera specified. Assuming not calibrated')
			self.cam_right_calibration = CameraInfo()

		self.load_dir = os.path.expanduser(rospy.get_param('/data_load_dir', '~/miro/data'))
		if self.load_dir[-1] != '/':
			self.load_dir += '/'

		self.resize = image_processing.make_size(height=rospy.get_param('/image_resize_height', None), width=rospy.get_param('/image_resize_width', None))
		if self.resize[0] is None and self.resize[1] is None:
			self.resize = None

		self.subsampling = rospy.get_param('/image_subsampling', 1)
		self.patch_size = image_processing.parse_patch_size_parameter(rospy.get_param('/patch_size', (9,9)))

		self.use_old_dataset_format = rospy.get_param('~use_old_dataset_format', False)
		self.use_depth = rospy.get_param('~use_depth', False)
		self.use_middle_weighting = rospy.get_param('~use_middle_weighting', False)

		self.full_images = []
		self.norm_images = []

		self.save_dir = os.path.expanduser(rospy.get_param('/data_save_dir','~/miro/data'))
		if self.save_dir[-1] != '/':
			self.save_dir += '/'
		if not os.path.isdir(self.save_dir):
			os.makedirs(self.save_dir)

		self.match_number = 0

	def setup_publishers(self):
		pass

	def setup_subscribers(self):
		self.service = rospy.Service('match_image', ImageMatch, self.match_image)
		# For Multi-Robot System
		self.sub_teacher_full_image = rospy.Subscriber("teacher_full_img", Image, self.append_teacher_full_image, queue_size=1, buff_size=2**22)
		self.sub_teacher_norm_image = rospy.Subscriber("teacher_norm_img", Image, self.append_teacher_norm_image, queue_size=1, buff_size=2**22)

	def append_teacher_full_image(self, msg):
		self.full_images.append(image_processing.msg_to_image(msg))

	def append_teacher_norm_image(self, msg):
		self.norm_images.append(image_processing.msg_to_image(msg))

	def clamp_search_range_to_bounds(self, index, half_search_range):
		start_range = max(0, index - half_search_range)
		end_range = min(len(self.images), index + half_search_range + 1)
		return start_range, end_range

	def match_image(self, request):
		# Added in to wait till there is enough data before proceeding to follow
		self.images = self.norm_images
		while(len(self.images) - request.imageIndex.data < self.frames_delay):
			pass

		image = image_processing.msg_to_image(request.normalisedImage)
		image_index = request.imageIndex.data
		start_range, end_range = self.clamp_search_range_to_bounds(image_index, request.searchRange.data)
		centre_image_index = image_index - start_range
		match_data = [[] for i in range(end_range - start_range)]
		debug_image = None
		for i in range(end_range - start_range):
			img_index = start_range + i
			if i == centre_image_index:
				offset, corr, debug_image = image_processing.xcorr_match_images_debug(self.images[img_index], image, self.subsampling)
				match_data[i] = (offset, corr)
			else:
				match_data[i] = image_processing.xcorr_match_images(self.images[img_index], image, self.subsampling)
		offsets_data = [int(match[0]) for match in match_data]
		correlations_data = [match[1] for match in match_data]
		offset = offsets_data[centre_image_index]
		correlation = correlations_data[centre_image_index]

		correlation_bar_height = 5
		correlation_bar = np.uint8(np.tile(255.0*np.array(correlations_data)[np.arange(debug_image.shape[1]) * int(len(correlations_data) / debug_image.shape[1])].reshape(1,-1,1), (correlation_bar_height,1,3)))

		debug_image = np.vstack((debug_image, correlation_bar))

		if debug_image is not None:
			self.pub_image_match_debug.publish(image_processing.image_to_msg(debug_image,'bgr8'))
			timestamp = datetime.datetime.now().strftime(r'%H-%M-%S-%f')
			cv2.imwrite(self.save_dir+'%06d_%s.png' % (self.match_number, timestamp), debug_image)
		else:
			raise RuntimeError('image matcher - debug_image is None but it should always be assigned.')

		self.match_number += 1

		# self.current_position = best_index + start_search_range
		offsets = Int32MultiArray()
		offsets.layout.dim = [MultiArrayDimension(size=len(match_data))]
		offsets.data = offsets_data
		correlations = Float32MultiArray()
		correlations.layout.dim = [MultiArrayDimension(size=len(match_data))]
		correlations.data = correlations_data

		return ImageMatchResponse(offsets, correlations)


if __name__ == "__main__":
	rospy.init_node("image_matcher")
	matcher = image_matcher()
	rospy.spin()
