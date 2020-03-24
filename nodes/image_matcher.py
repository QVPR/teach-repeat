#!/usr/bin/python

import rospy
import numpy as np
import cv2
import os
import pickle
import time
import json
from rospy_message_converter import message_converter
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
import tf_conversions


import image_processing
from miro_teach_repeat.msg import ImageAndPose
from miro_teach_repeat.srv import ImageMatch


class miro_image_matcher:

	def __init__(self):		
		self.pub_image_match_debug = rospy.Publisher('/miro/match_image_debug', Image, queue_size=0)

		self.save_dir = os.path.expanduser(rospy.get_param('~save_dir', '~/miro/data'))
		if self.save_dir[-1] != '/':
			self.save_dir += '/'

		image_files = [self.save_dir+f for f in os.listdir(self.save_dir) if f[-10:] == '_image.pkl']
		image_files.sort()
		pose_files = [self.save_dir+f for f in os.listdir(self.save_dir) if f[-9:] == '_pose.txt']
		pose_files.sort()

		print('loading images...')
		self.images = self.load_images(image_files)
		print('loading poses...')
		self.poses = self.load_poses(pose_files)
		print('loading complete: %d images and %d poses' % (len(self.images), len(self.poses)))

		# setup the service listener for other nodes to get images matched
		self.service = rospy.Service('/miro/match_image', ImageMatch, self.match_image)

	def load_images(self, image_files):
		return [pickle.loads(self.read_file(f)) for f in image_files]
		# return [cv2.resize(pickle.loads(self.read_file(f)), (80,45), interpolation=cv2.INTER_NEAREST) for f in image_files]

	def load_poses(self, pose_files):
		return [message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',json.loads(self.read_file(f))) for f in pose_files]

	def read_file(self, filename):
		with open(filename, 'r') as f:
			data = f.read()
		return data

	def match_image(self, request):
		image = image_processing.msg_to_image(request.normalisedImage)
		vertical_cutoff = 0.8
		match_data = [image_processing.xcorr_match_images(ref_img, image, template_proportion=0.7, vertical_cutoff=vertical_cutoff) for ref_img in self.images]
		best_index = np.argmax([m[1] for m in match_data])
		if best_index == len(self.images)-1:
			delta_pose = Pose()
			delta_pose.orientation.w = 1
		else:
			img_frame = tf_conversions.fromMsg(self.poses[best_index])
			next_frame = tf_conversions.fromMsg(self.poses[best_index+1])
			delta_pose = tf_conversions.toMsg(img_frame.Inverse() * next_frame)
		# TODO - offset this pose difference by the relative orientation found in the image

		debug_image = np.concatenate((image, self.images[best_index]), axis=1)
		debug_image = np.uint8(255.0 * (1 + debug_image) / 2.0)
		debug_image = cv2.merge((debug_image, debug_image, debug_image))
		cv2.line(debug_image, (int(match_data[best_index][0]+self.images[best_index].shape[1]/2),0), (int(match_data[best_index][0]+self.images[best_index].shape[1]/2),self.images[best_index].shape[0]), (0,255,0))
		cv2.line(debug_image, (int(self.images[best_index].shape[1]+image.shape[1]/2),0), (int(self.images[best_index].shape[1]+image.shape[1]/2),self.images[best_index].shape[0]), (0,255,0))
		cv2.line(debug_image, (0,int(vertical_cutoff*debug_image.shape[0])), (debug_image.shape[1]-1,int(vertical_cutoff*debug_image.shape[0])), (255,0,0))
		self.pub_image_match_debug.publish(image_processing.image_to_msg(debug_image,'bgr8'))

		return delta_pose


if __name__ == "__main__":

	rospy.init_node("miro_image_matcher")
	integrator = miro_image_matcher()
	rospy.spin()
