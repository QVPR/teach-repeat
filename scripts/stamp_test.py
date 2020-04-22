#!/usr/bin/python

import rospy
from sensor_msgs.msg import CompressedImage
import tf_conversions
import message_filters
from std_msgs.msg import Float64

import math
import time

class miro_stamp_test:

	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):	
		self.last_left_stamp = None
		self.last_right_stamp = None
		self.N_sync = 0
		self.N_L = 0
		self.N_R = 0
		self.time_offset = 0

	def setup_publishers(self):	
		self.pub_delta_t_real = rospy.Publisher('/miro/delta_t_real', Float64, queue_size=1)

	def setup_subscribers(self):
		# subscribe to the images from both cameras
		self.sub_image_left = message_filters.Subscriber("/miro/sensors/cam/left/compressed", CompressedImage, queue_size=1, buffer_size=2**24)
		self.sub_image_right = message_filters.Subscriber("/miro/sensors/cam/right/compressed", CompressedImage, queue_size=1, buffer_size=2**24)
		self.sub_images = message_filters.ApproximateTimeSynchronizer((self.sub_image_left, self.sub_image_right), 10, 1.0/30.0)
		self.sub_images.registerCallback(self.process_image_data)
		self.sub_image_left.registerCallback(self.process_left_image_data)
		self.sub_image_right.registerCallback(self.process_right_image_data)
		
	def process_left_image_data(self, msg):
		self.N_L += 1
		if self.N_L == 1:
			self.time_offset = rospy.Time.now().to_sec() - msg.header.stamp.to_sec()

	def process_right_image_data(self, msg):
		self.N_R += 1

	def process_image_data(self, msg_left, msg_right):
		self.N_sync += 1
		Qstr = '[' + ','.join([str(len(q)) for q in self.sub_images.queues]) + ']'
		print('%d: Q=%s N=[%d,%d]' % (self.N_sync,Qstr,self.N_L,self.N_R))
		delta_t_real = rospy.Time.now().to_sec() - msg_left.header.stamp.to_sec() - self.time_offset
		print('delta t real = %f s' % delta_t_real)
		self.pub_delta_t_real.publish(Float64(data=delta_t_real))

	def stop(self):
		print('left [%d], right [%d], synched [%d]' % (self.N_L, self.N_R, self.N_sync))

if __name__ == "__main__":
	rospy.init_node("img_stamp_tester")
	stamp_tester = miro_stamp_test()
	rospy.on_shutdown(stamp_tester.stop)
	rospy.spin()