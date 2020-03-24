#!/usr/bin/python

import rospy
import numpy as np
import cv2
import os
import pickle
import datetime
from rospy_message_converter import message_converter
from geometry_msgs.msg import Pose

import math
import json

import image_processing
from miro_teach_repeat.msg import ImageAndPose
from miro_teach_repeat.srv import ImageMatch


class miro_image_match:

	def __init__(self):		
		# setup the service listener for other nodes to get images matched
		self.service = rospy.Service('match_image', ImageMatch, self.match_image)

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

	def load_images(self, image_files):
		return [pickle.loads(self.read_file(f)) for f in image_files]

	def load_poses(self, pose_files):
		return [message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',json.loads(self.read_file(f))) for f in pose_files]

	def read_file(self, filename):
		with open(filename, 'r') as f:
			data = f.read()
		return data

	def match_image(self, request):
		match_data = [image_processing.scan_horizontal_SAD_match(ref_img, request.normalisedImage, step_size=1) for ref_img in self.images]
		best_index = np.argmin([m[0] for m in match_data])
		if best_index == len(images)-1:
			delta_pose = Pose()
			delta_pose.orientation.w = 1
		else:
			img_frame = tf_conversions.fromMsg(self.poses[best_index])
			next_frame = tf_conversions.fromMsg(self.poses[best_index+1])
			delta_pose = tf_conversions.toMsg(img_frame.Inverse() * next_frame)
		# TODO - offset this pose difference by the relative orientation found in the image
		return delta_pose


if __name__ == "__main__":

	rospy.init_node("miro_image_match")
	integrator = miro_image_match()
	rospy.spin()
