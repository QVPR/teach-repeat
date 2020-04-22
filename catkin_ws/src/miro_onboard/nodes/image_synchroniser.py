#!/usr/bin/python

import rospy
import math
import time
from sensor_msgs.msg import CompressedImage
import tf_conversions
import message_filters
from std_msgs.msg import Float64

from miro_onboard.msg import CompressedImageSynchronised

class miro_image_synchroniser:

	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):
		pass

	def setup_publishers(self):	
		self.pub_images_synchronised = rospy.Publisher("/miro/senors/cam/both/compressed", CompressedImageSynchronised, queue_size=10)

	def setup_subscribers(self):
		# subscribe to the images from both cameras
		self.sub_image_left = message_filters.Subscriber("/miro/sensors/cam/left/compressed", CompressedImage, queue_size=10, buffer_size=2**22)
		self.sub_image_right = message_filters.Subscriber("/miro/sensors/cam/right/compressed", CompressedImage, queue_size=10, buffer_size=2**22)
		self.sub_images = message_filters.ApproximateTimeSynchronizer((self.sub_image_left, self.sub_image_right), 10, 1.0/30.0)
		self.sub_images.registerCallback(self.process_image_data)

	def process_image_data(self, msg_left, msg_right):
		image_sync_msg = CompressedImageSynchronised()
		image_sync_msg.left = msg_left
		image_sync_msg.right = msg_right
		self.pub_images_synchronised.publish(image_sync_msg)

if __name__ == "__main__":
	rospy.init_node("miro_image_synchroniser")
	syncrhoniser = miro_image_synchroniser()
	rospy.spin()