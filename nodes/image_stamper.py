#!/usr/bin/python

import rospy
from sensor_msgs.msg import CompressedImage

class miro_image_stamper:

	def __init__(self):
		# subscribe to image
		self.sub_image_left = rospy.Subscriber("/miro/sensors/caml/compressed", CompressedImage, self.process_image_data_left, queue_size=1)
		self.sub_image_right = rospy.Subscriber("/miro/sensors/camr/compressed", CompressedImage, self.process_image_data_right, queue_size=1)
		# publish stamped image
		self.pub_image_left = rospy.Publisher("/miro/sensors/caml_stamped/compressed", CompressedImage, queue_size=0)
		self.pub_image_right = rospy.Publisher("/miro/sensors/camr_stamped/compressed", CompressedImage, queue_size=0)

	def process_image_data_left(self, msg):
		msg.header.stamp = rospy.Time.now()
		if len(msg.data) == 0:
			rospy.logwarn("[Image Stamper] received empty compressed image - dropping message")
			return
		self.pub_image_left.publish(msg)

	def process_image_data_right(self, msg):
		msg.header.stamp = rospy.Time.now()
		if len(msg.data) == 0:
			rospy.logwarn("[Image Stamper] received empty compressed image - dropping message")
			return
		self.pub_image_right.publish(msg)

if __name__ == "__main__":

	rospy.init_node("miro_image_stamper")
	stamper = miro_image_stamper()
	rospy.spin()

