#!/usr/bin/python

import rospy
import numpy as np
import cv2
import os
import pickle
import datetime
from rospy_message_converter import message_converter

import math
import json

import image_processing
from miro_teach_repeat.msg import ImageAndPose


class miro_image_match:

	def __init__(self):		
		# subscribe to image and pose pair
		self.pub_image_pose = rospy.Subscriber("/miro/image_pose", ImageAndPose, self.process_image_and_pose, queue_size=1)
		self.save_dir = os.path.expanduser(rospy.get_param('~save_dir', '~/miro/data'))
		image_files = [f for f in os.listdir(self.save_dir) if f[-10:] == '_image.pkl']
		image_files.sort()
		pose_files = [f for f in os.listdir(self.save_dir) if f[-9:] == '_pose.txt']
		pose_files.sort()

		self.images = self.load_images(image_files)
		self.poses = self.load_poses(pose_files)

	def load_images(self, image_files):
		return [pickle.loads(self.read_file(f)) for f in image_files]

	def load_poses(self, pose_files):
		return [message_converter.convert_dictionary_to_ros_message(json.loads(self.read_file(f))) for f in pose_files]

	def read_file(self, filename):
		with open(filename, 'r') as f:
			data = f.read()
		return data

	def match_images(self, image):
		match_data = [image_processing.scan_horizontal_SAD_match(ref_img, image, step_size=1) for ref_img in self.images]
		best_error = min([m[0] for m in match_data])
		# TODO


if __name__ == "__main__":

	rospy.init_node("miro_image_match")
	integrator = miro_image_match()
	rospy.spin()
