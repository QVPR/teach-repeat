#!/usr/bin/python

import rospy
import numpy as np
import cv2
import os
import pickle
import datetime
import time
import math
import json
from rospy_message_converter import message_converter
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Float64
import message_filters

import image_processing
from miro_onboard.msg import CompressedImageSynchronised
from miro_teach_repeat.srv import ImageMatch
from miro_teach_repeat.srv import PoseOffset
class miro_localiser:

	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):	
		self.resize = image_processing.make_size(height=rospy.get_param('/image_resize_height', None), width=rospy.get_param('/image_resize_width', None))
		if self.resize[0] is None and self.resize[1] is None:
			self.resize = None

		self.image_offset_gain = rospy.get_param('~image_offset_gain', 2.3)
		
		self.last_image = None
		self.first_img_seq = 0

		self.save_dir = os.path.expanduser(rospy.get_param('/miro_data_save_dir','~/miro/data/follow-straight_tests/5'))
		if self.save_dir[-1] != '/':
			self.save_dir += '/'
		if not os.path.isdir(self.save_dir):
			os.makedirs(self.save_dir)
		if not os.path.isdir(self.save_dir+'full/'):
			os.makedirs(self.save_dir+'full/')
		if not os.path.isdir(self.save_dir+'norm/'):
			os.makedirs(self.save_dir+'norm/')

		self.Nleft = 0
		self.Nright = 0
		self.last_stamp = None
		self.time_offset = 0
		
	def setup_publishers(self):	
		rospy.wait_for_service('/miro/match_image')
		self.match_image = rospy.ServiceProxy('/miro/match_image', ImageMatch, persistent=True)
		self.pub_delta_t_real = rospy.Publisher('/miro/delta_t_real', Float64, queue_size=1)

	def setup_subscribers(self):
		# subscribe to the images from both cameras
		self.sub_images = rospy.Subscriber('/miro/sensors/cam/both/compressed', CompressedImageSynchronised, self.process_image_data, queue_size=1, buff_size=2**22)
		self.service = rospy.Service('/miro/get_image_pose_offset', PoseOffset, self.calculate_image_pose_offset)

	def process_image_data(self, msg):
		n = msg.left.header.seq
		if self.last_image is None:
			self.first_img_seq = n
		# if self.last_stamp is not None:
			# print('delta t = %f s' % (msg.left.header.stamp - self.last_stamp).to_sec())
			# delta_t_real = rospy.Time.now().to_sec() - msg.left.header.stamp.to_sec() - self.time_offset
			# print('delta t real = %f s' % delta_t_real)
			# self.pub_delta_t_real.publish(Float64(data=delta_t_real))
		# else: 
			# self.time_offset = (rospy.Time.now() - msg.left.header.stamp).to_sec()
		n -= self.first_img_seq

		full_image = image_processing.stitch_stereo_image_message(msg.left, msg.right, compressed=True)
		normalised_image = image_processing.patch_normalise_image(full_image, (9,9), resize=self.resize)
		
		cv2.imwrite(self.save_dir+('full/%06d.png' % n), full_image)
		cv2.imwrite(self.save_dir+('norm/%06d.png' % n), np.uint8(255.0 * (1 + normalised_image) / 2.0))

		self.last_image = normalised_image
		# self.last_stamp = msg.left.header.stamp

	def calculate_image_pose_offset(self, srv):
		if self.last_image is not None:
			image_offset = self.match_image(image_processing.image_to_msg(self.last_image), srv.goalIndex).pixelOffset.data

			# positive image offset: query image is shifted left from reference image
			# this means we have done a right (negative turn) which we should correct with a positive turn
			# positive offset -> positive turn, gain = positive
			# (normalise the pixel offset by the width of the image)
			return Float64(data=self.image_offset_gain * float(image_offset) / self.last_image.shape[1])
		else:
			raise RuntimeError('Localiser: tried to localise before image data is received!')


if __name__ == "__main__":
	rospy.init_node("miro_localiser")
	localiser = miro_localiser()
	rospy.spin()