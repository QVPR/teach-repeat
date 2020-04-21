#!/usr/bin/python

import rospy
from sensor_msgs.msg import CompressedImage
import tf_conversions
import message_filters

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

	def setup_publishers(self):	
		pass

	def setup_subscribers(self):
		self.sub_left = rospy.Subscriber("/miro/sensors/cam/left/compressed", CompressedImage, self.process_left_image_data)
		self.sub_left = rospy.Subscriber("/miro/sensors/cam/right/compressed", CompressedImage, self.process_right_image_data)

		# subscribe to the images from both cameras
		self.sub_image_left = message_filters.Subscriber("/miro/sensors/cam/left/compressed", CompressedImage, queue_size=10)
		self.sub_image_right = message_filters.Subscriber("/miro/sensors/cam/right/compressed", CompressedImage, queue_size=10)
		self.sub_images = message_filters.ApproximateTimeSynchronizer((self.sub_image_left, self.sub_image_right), 100, 1.0/30.0)
		self.sub_images.registerCallback(self.process_image_data)
		
	def process_left_image_data(self, msg):
		self.N_L += 1
		stamp = msg.header.stamp
		if self.last_right_stamp is not None:
			print('[%d] L - %f [diff = %f s]' % (self.N_L, stamp.to_sec(), (stamp - self.last_right_stamp).to_sec()))
		else:
			print('[%d] L - %f' % (self.N_L, msg.header.stamp.to_sec()))
		self.last_left_stamp = stamp

	def process_right_image_data(self, msg):
		self.N_R += 1
		stamp = msg.header.stamp
		if self.last_left_stamp is not None:
			print('[%d] R - %f [diff = %f s]' % (self.N_R, stamp.to_sec(), (stamp - self.last_left_stamp).to_sec()))
		else:
			print('[%d] R - %f' % (self.N_R, msg.header.stamp.to_sec()))
		self.last_right_stamp = stamp

	def process_image_data(self, msg_left, msg_right):
		self.N_sync += 1
		print('%d: diff = %f. Q = [%d,%d]' % (self.N_sync, abs(msg_left.header.stamp - msg_right.header.stamp).to_sec(),len(self.sub_images.queues[0]),len(self.sub_images.queues[1])))

	def stop(self):
		print('left [%d], right [%d], synched [%d]' % (self.N_L, self.N_R, self.N_sync))

if __name__ == "__main__":
	rospy.init_node("img_stamp_diff")
	collector = miro_stamp_test()
	rospy.on_shutdown(collector.stop)
	rospy.spin()