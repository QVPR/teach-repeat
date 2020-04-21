#!/usr/bin/python

import rospy
import numpy as np
import cv2
import os
import pickle
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

import image_processing
from miro_teach_repeat.srv import ImageMatch

# SEARCH_SIZE = 3

class miro_image_matcher:

	def __init__(self):		
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):
		self.pub_image_match_debug = rospy.Publisher('/miro/match_image_debug', Image, queue_size=0)

		self.load_dir = os.path.expanduser(rospy.get_param('/miro_data_load_dir', '~/miro/data'))
		if self.load_dir[-1] != '/':
			self.load_dir += '/'

		self.resize = image_processing.make_size(height=rospy.get_param('/image_resize_height', None), width=rospy.get_param('/image_resize_width', None))
		if self.resize[0] is None and self.resize[1] is None:
			self.resize = None

		image_files = [self.load_dir+f for f in os.listdir(self.load_dir) if f[-10:] == '_image.pkl']
		image_files.sort()

		print('loading images...')
		self.images = self.load_images(image_files)
		print('loading complete: %d images' % (len(self.images)))

		self.save_dir = os.path.expanduser(rospy.get_param('/miro_data_save_dir','~/miro/data'))
		if self.save_dir[-1] != '/':
			self.save_dir += '/'
		if not os.path.isdir(self.save_dir):
			os.makedirs(self.save_dir)

	def setup_publishers(self):
		pass

	def setup_subscribers(self):
		self.service = rospy.Service('/miro/match_image', ImageMatch, self.match_image)

	def load_images(self, image_files):
		images = [pickle.loads(self.read_file(f)) for f in image_files]
		if self.resize is None or self.resize == images[0].shape[:2]:
			return images
		else:
			# need to flip the order of resize for opencv
			return [cv2.resize(image, self.resize[::-1], interpolation=cv2.INTER_NEAREST) for image in images]

	def read_file(self, filename):
		with open(filename, 'r') as f:
			data = f.read()
		return data

	def match_image(self, request):
		image = image_processing.msg_to_image(request.normalisedImage)
		# start_search_range = max(0, self.current_position - SEARCH_SIZE)
		# end_search_range = min(len(self.images), self.current_position + SEARCH_SIZE)
		# match_data = [image_processing.xcorr_match_images(ref_img, image) for ref_img in self.images[start_search_range:end_search_range]]
		# match_data = [image_processing.xcorr_match_images(ref_img, image) for ref_img in self.images]
		# best_index = np.argmax([m[1] for m in match_data])
		match_data = [image_processing.xcorr_match_images(self.images[request.imageIndex.data], image)]
		best_index = 0

		offset = match_data[best_index][0]

		debug_image = np.concatenate((image, self.images[request.imageIndex.data]), axis=0)
		debug_image = np.uint8(255.0 * (1 + debug_image) / 2.0)
		debug_image = cv2.merge((debug_image, debug_image, debug_image))
		cv2.line(debug_image, (int(-offset+image.shape[1]/2),0), (int(-offset+image.shape[1]/2),image.shape[0]-1), (0,255,0))
		cv2.line(debug_image, (int(image.shape[1]/2),image.shape[0]), (int(image.shape[1]/2), 2*image.shape[0]), (200,0,0))
		self.pub_image_match_debug.publish(image_processing.image_to_msg(debug_image,'bgr8'))

		cv2.imwrite(self.save_dir+'%06d.png' % request.imageIndex.data, debug_image)

		# self.current_position = best_index + start_search_range
		return Int32(data=offset)


if __name__ == "__main__":
	rospy.init_node("miro_image_matcher")
	matcher = miro_image_matcher()
	rospy.spin()