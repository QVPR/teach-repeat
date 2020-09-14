#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import os
import math
import subprocess
from sensor_msgs.msg import Image

import image_processing

class mono_depth_image_publisher:
	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):
		self.image_file = rospy.get_param('/image_file', None)
		
		if self.image_file is None:
			rospy.logerr('no image filename provided')

		self.depth_image_file = self.image_file[:self.image_file.index('.')] + '_disp.npy'
		
		if os.path.isfile(self.image_file):
			self.image = cv2.imread(self.image_file, cv2.IMREAD_GRAYSCALE)
		else:
			rospy.logerr('image does not exist provided')
		
		if not os.path.isfile(self.depth_image_file):
			output = subprocess.check_output(['python','~/monopath2/test_image.py','--image_path',self.image_file,'--model_name','mono+stereo_640x192'])
			print(output)
		self.depth_image = np.load(self.depth_image_file).squeeze()

		if self.depth_image.shape != self.image.shape:
			self.depth_image = cv2.resize(self.depth_image, self.image.shape[::-1])

		self.image_msg = image_processing.image_to_msg(self.image)
		self.depth_image_msg = image_processing.image_to_msg(self.depth_image)
		self.image_msg.header.frame_id = 'cam'
		self.depth_image_msg.header.frame_id = 'cam'
		
	def setup_publishers(self):	
		self.image_pub = rospy.Publisher('/image', Image, queue_size=1)
		self.depth_pub = rospy.Publisher('/depth_image', Image, queue_size=1)

	def setup_subscribers(self):
		pass

	def pub_images(self):
		self.image_msg.header.stamp = rospy.Time.now()
		self.depth_image_msg.header.stamp = rospy.Time.now()

		self.image_pub.publish(self.image_msg)
		self.depth_pub.publish(self.depth_image_msg)

if __name__ == "__main__":
	rospy.init_node("mono_depth_image_publisher")
	depth_pub = mono_depth_image_publisher()
	r = rospy.Rate(1)
	while not rospy.is_shutdown():
		depth_pub.pub_images()
		r.sleep()
