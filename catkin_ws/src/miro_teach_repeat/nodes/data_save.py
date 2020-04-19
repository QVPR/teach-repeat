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


class miro_data_save:

	def __init__(self):	
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):
		self.save_id = 0
		self.save_dir = os.path.expanduser(rospy.get_param('~save_dir', '~/miro/data'))
		self.timestamp_dir = rospy.get_param('~timestamp_folder', False)
		if self.save_dir[-1] != '/':
			self.save_dir += '/'
		if self.timestamp_dir:
			self.save_dir += datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S/')
		if not os.path.isdir(self.save_dir):
			os.makedirs(self.save_dir)

		self.resize = image_processing.make_size(height=rospy.get_param('~image_resize_height', None), width=rospy.get_param('~image_resize_width', None))
		if self.resize[0] is None and self.resize[1] is None:
			self.resize = None

	def setup_publishers(self):
		pass

	def setup_subscribers(self):
		self.sub_image_pose = rospy.Subscriber("/miro/image_pose", ImageAndPose, self.process_image_and_pose, queue_size=1)
		
	def process_image_and_pose(self, msg):
		image = image_processing.stitch_stereo_image(image_processing.compressed_msg_to_image(msg.image_left), image_processing.compressed_msg_to_image(msg.image_right))
		pose = msg.pose
		id = "%06d" % (self.save_id)
		normalised_image = image_processing.patch_normalise_image(image, (9,9), resize=self.resize)
		message_as_text = json.dumps(message_converter.convert_ros_message_to_dictionary(pose))
		image_as_text = pickle.dumps(normalised_image)


		cv2.imwrite(self.save_dir+id+'_left.png', np.uint8(image_processing.compressed_msg_to_image(msg.image_left)))
		cv2.imwrite(self.save_dir+id+'_right.png', np.uint8(image_processing.compressed_msg_to_image(msg.image_right)))
		cv2.imwrite(self.save_dir+id+'_full.png', np.uint8(image))
		cv2.imwrite(self.save_dir+id+'_thumbnail.png', np.uint8(255.0 * (1 + normalised_image) / 2.0))
		with open(self.save_dir+id+'_pose.txt', 'w') as pose_file:
			pose_file.write(message_as_text)
		with open(self.save_dir+id+'_image.pkl', 'w') as image_file:
			image_file.write(image_as_text)
		self.save_id += 1


if __name__ == "__main__":

	rospy.init_node("miro_data_save")
	saver = miro_data_save()
	rospy.spin()
