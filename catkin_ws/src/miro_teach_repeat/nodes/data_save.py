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
		# subscribe to image and pose pair
		self.sub_image_pose = rospy.Subscriber("/miro/image_pose", ImageAndPose, self.process_image_and_pose, queue_size=1)
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
		
	def process_image_and_pose(self, msg):
		image = msg.image
		pose = msg.pose
		id = "%06d" % (self.save_id)
		normalised_image = image_processing.patch_normalise_msg(image, (9,9), compressed=True, resize=self.resize)
		message_as_text = json.dumps(message_converter.convert_ros_message_to_dictionary(pose))
		image_as_text = pickle.dumps(normalised_image)

		cv2.imwrite(self.save_dir+id+'_thumbnail.png', np.uint8(255.0 * (1 + normalised_image) / 2.0))
		with open(self.save_dir+id+'_pose.txt', 'w') as pose_file:
			pose_file.write(message_as_text)
		with open(self.save_dir+id+'_image.pkl', 'w') as image_file:
			image_file.write(image_as_text)
		self.save_id += 1


if __name__ == "__main__":

	rospy.init_node("miro_data_save")
	integrator = miro_data_save()
	rospy.spin()
