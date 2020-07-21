#!/usr/bin/python

import rospy
import numpy as np
import cv2
import os
import pickle
import yaml
import json
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32MultiArray, Float32MultiArray, MultiArrayDimension

import image_processing
from miro_teach_repeat.srv import ImageMatch, ImageMatchResponse

def get_image_files_from_dir(file_dir, file_ending):
	files = [f for f in os.listdir(file_dir) if f.endswith(file_ending)]
	files.sort()
	return [file_dir+f for f in files]

class image_matcher:

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

		print('loading images...')
		if self.use_old_dataset_format:
			if os.path.isdir(self.load_dir+'left/') and os.path.isdir(self.load_dir+'right/'):
				image_files = zip(get_image_files_from_dir(self.load_dir+'left/', '.png'), get_image_files_from_dir(self.load_dir+'right/', '.png'))
				self.images = self.load_images_left_right(image_files)
			else:
				rospy.logerr('[Image matcher] use_old_dataset_format selected, but dataset is not in old format (seperate folders for left and right images)')
		else:
			if os.path.exists(self.load_dir+'params.txt'):
				params = json.loads(self.read_file(self.load_dir+'params.txt'))
				# if dataset collection params match what we want, use pre-processed images
				if self.resize == params['resize'] and self.patch_size == params['patch_size']:
					image_files = get_image_files_from_dir(self.load_dir+'norm/', '.png')
					self.images = self.load_images(image_files)
				else:
					image_files = get_image_files_from_dir(self.load_dir+'full/', '.png')
					self.images = self.load_images_resize_norm(image_files)
			else:
				image_files = get_image_files_from_dir(self.load_dir+'full/', '.png')
				self.images = self.load_images_resize_norm(image_files)

		if self.use_depth:
			# get depth maps from full images (not patch normed)
			self.images = self.weight_images_depth(self.images, get_image_files_from_dir(self.load_dir+'full/', '.png'))
		if self.use_middle_weighting:
			self.images = self.weight_images_middle(self.images)
		print('loading complete: %d images' % (len(self.images)))

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

	def load_image_left_right(self, file_left, file_right):
		image_left = image_processing.grayscale(cv2.imread(file_left))
		image_right = image_processing.grayscale(cv2.imread(file_right))
		image_both, _ = image_processing.rectify_stitch_stereo_image(image_left, image_right, self.cam_left_calibration, self.cam_right_calibration)
		return image_processing.patch_normalise_pad(cv2.resize(image_both,  self.resize[::-1], interpolation=cv2.INTER_AREA), self.patch_size)

	def load_images_left_right(self, image_files):
		return [self.load_image_left_right(*image_pair) for image_pair in image_files]

	def load_images_resize_norm(self, image_files):
		images = self.load_images(image_files)
		return [image_processing.patch_normalise_pad(cv2.resize(image,  self.resize[::-1], interpolation=cv2.INTER_AREA), self.patch_size) for image in images]

	def load_images(self, image_files):
		return [cv2.imread(image_file, cv2.IMREAD_GRAYSCALE) for image_file in image_files]

	def weight_images_depth(self, images, image_files):
		depth_images = [np.load(f[:-4]+'_disp.npy').squeeze() for f in image_files]
		depth_images = [np.clip(cv2.resize(image, self.resize[::-1], interpolation=cv2.INTER_AREA), 0, 1) for image in depth_images]
		# depth_images = [np.maximum(image, image > 0.5) for image in depth_images]
		return [image * depth for image, depth in zip(images, depth_images)]

	def weight_images_middle(self, images):
		horizontal_weighting = np.clip(1.0 - 1.0*(np.abs(np.arange(self.resize[1]) - (self.resize[1]/2.)).reshape((1,-1)) / (self.resize[1]/2.))**2, 0, 1)
		vertical_weighting = np.clip(1.0 + 0*np.linspace(0,1,self.resize[0]).reshape(-1,1)**0.5, 0, 1)
		image_weighting = vertical_weighting * horizontal_weighting
		return [image * image_weighting for image in images]

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
	rospy.init_node("image_matcher")
	matcher = image_matcher()
	rospy.spin()