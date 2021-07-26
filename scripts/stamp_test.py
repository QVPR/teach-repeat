#!/usr/bin/env python

import rospy
import math
import time
from sensor_msgs.msg import CompressedImage
import tf_conversions
import message_filters
from std_msgs.msg import Float64

from miro_onboard.msg import CompressedImageSynchronised

class miro_stamp_test:

	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):	
		self.N = 0
		self.time_offset = None

	def setup_publishers(self):	
		self.pub_delta_t_real = rospy.Publisher('/miro/delta_t_real', Float64, queue_size=1)

	def setup_subscribers(self):
		self.sub_images = rospy.Subscriber('/miro/sensors/cam/both/compressed', CompressedImageSynchronised, queue_size=1, buff_size=2**22)

	def process_image_data(self, msg):
		if self.N == 0:
			self.time_offset = (rospy.Time.now() - msg.left.header.stamp).to_sec()
		self.N += 1
		
		delta_t_real = (rospy.Time.now() - msg.left.header.stamp).to_sec() - self.time_offset
		self.pub_delta_t_real.publish(Float64(data=delta_t_real))

if __name__ == "__main__":
	rospy.init_node("img_stamp_tester")
	stamp_tester = miro_stamp_test()
	rospy.spin()