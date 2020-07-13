#!/usr/bin/python

import rospy
import numpy as np
import cv2
import os
import pickle
import yaml
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32MultiArray, Float32MultiArray, MultiArrayDimension

import image_processing
from miro_teach_repeat.srv import ImageMatch, ImageMatchResponse

def get_image_files_from_dir(file_dir, file_ending):
	files = [f for f in os.listdir(file_dir) if f.endswith(file_ending)]
	files.sort()
	return [file_dir+f for f in files]

class miro_image_matcher:

	def __init__(self):		
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):
		self.pub_image_match_debug = rospy.Publisher('/miro/match_image_debug', Image, queue_size=0)

		self.left_cal_file = rospy.get_param('/calibration_file_left', None)
		self.right_cal_file = rospy.get_param('/calibration_file_right', None)

		if self.left_cal_file is not None:
			with open(self.left_cal_file,'r') as f:
				self.cam_left_calibration = image_processing.yaml_to_camera_info(yaml.load(f.read()))
		else:
			rospy.loginfo('[Image Matcher] no calibration file for left camera specified. Assuming not calibrated')
			self.cam_left_calibration = CameraInfo()
		
		if self.right_cal_file is not None:
			with open(self.right_cal_file,'r') as f:
				self.cam_right_calibration = image_processing.yaml_to_camera_info(yaml.load(f.read()))
		else:
			rospy.loginfo('[Image Matcher] no calibration file for right camera specified. Assuming not calibrated')
			self.cam_right_calibration = CameraInfo()

		self.load_dir = os.path.expanduser(rospy.get_param('/miro_data_load_dir', '~/miro/data'))
		if self.load_dir[-1] != '/':
			self.load_dir += '/'

		self.resize = image_processing.make_size(height=rospy.get_param('/image_resize_height', None), width=rospy.get_param('/image_resize_width', None))
		if self.resize[0] is None and self.resize[1] is None:
			self.resize = None

		self.subsampling = rospy.get_param('/image_subsampling', 1)

		# image_files = [self.load_dir+f for f in os.listdir(self.load_dir) if f[-10:] == '_image.pkl']
		# image_files = [self.load_dir+'full/'+f for f in os.listdir(self.load_dir+'full/') if f[-4:] == '.png']
		image_files = zip(get_image_files_from_dir(self.load_dir+'left/', '.png'), get_image_files_from_dir(self.load_dir+'right/', '.png'))

		print('loading images...')
		self.images = self.load_images(image_files)
		print('loading complete: %d images' % (len(self.images)))

		self.save_dir = os.path.expanduser(rospy.get_param('/miro_data_save_dir','~/miro/data'))
		if self.save_dir[-1] != '/':
			self.save_dir += '/'
		if not os.path.isdir(self.save_dir):
			os.makedirs(self.save_dir)

		self.match_number = 0

	def setup_publishers(self):
		pass

	def setup_subscribers(self):
		self.service = rospy.Service('/miro/match_image', ImageMatch, self.match_image)

	def load_image(self, file_left, file_right):
		image_left = image_processing.grayscale(cv2.imread(file_left))
		image_right = image_processing.grayscale(cv2.imread(file_right))
		image_both, _ = image_processing.rectify_stitch_stereo_image(image_left, image_right, self.cam_left_calibration, self.cam_right_calibration)
		return image_processing.patch_normalise_pad(cv2.resize(image_both,  self.resize[::-1], interpolation=cv2.INTER_AREA), (9,9))

	def load_images(self, image_files):
		return [self.load_image(*image_pair) for image_pair in image_files]

	def read_file(self, filename):
		with open(filename, 'r') as f:
			data = f.read()
		return data

	def match_image(self, request):
		image = image_processing.msg_to_image(request.normalisedImage)
		image_index = request.imageIndex.data
		start_range = max(0, image_index - request.searchRange.data)
		end_range = min(len(self.images), image_index + request.searchRange.data + 1)
		best_index = image_index - start_range
		match_data = [[] for i in range(end_range - start_range)]
		debug_image = None
		for i in range(end_range - start_range):
			img_index = start_range + i
			if i == best_index:
				offset, corr, debug_image = image_processing.xcorr_match_images_debug(self.images[img_index], image, self.subsampling)
				match_data[i] = (offset, corr)
			else:
				match_data[i] = image_processing.xcorr_match_images(self.images[img_index], image, self.subsampling)
		offset = match_data[best_index][0]
		correlation = match_data[best_index][1]

		if debug_image is not None:
			self.pub_image_match_debug.publish(image_processing.image_to_msg(debug_image,'bgr8'))
			cv2.imwrite(self.save_dir+'%06d.png' % self.match_number, debug_image)
		else:
			raise RuntimeError('image matcher - debug_image is None but it should always be assigned.')

		image_as_text = pickle.dumps(image)
		with open(self.save_dir+('%06d_image.pkl' % self.match_number), 'w') as image_file:
			image_file.write(image_as_text)

		self.match_number += 1

		# self.current_position = best_index + start_search_range
		offsets = Int32MultiArray()
		offsets.layout.dim = [MultiArrayDimension(size=len(match_data))]
		offsets.data = [int(match[0]) for match in match_data]
		correlations = Float32MultiArray()
		correlations.layout.dim = [MultiArrayDimension(size=len(match_data))]
		correlations.data = [match[1] for match in match_data]

		return ImageMatchResponse(offsets, correlations)


if __name__ == "__main__":
	rospy.init_node("miro_image_matcher")
	matcher = miro_image_matcher()
	rospy.spin()