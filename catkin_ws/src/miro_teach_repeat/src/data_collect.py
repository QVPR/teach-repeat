#!/usr/bin/python

import rospy
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import tf_conversions

import cv2
import cv_bridge
import numpy as np
import math

class miro_integrate_odom:

	def __init__(self):		
		# subscribe to instantaneous odometry
		self.sub_odom = rospy.Subscriber("/miro/odom_integrated", Odometry, self.process_odom_data, queue_size=1)

		self.sub_image_left = rospy.Subscriber("/miro/sensors/caml/compressed", CompressedImage, self.process_image_data, queue_size=1)

		self.last_odom = None
		self.current_odom = None

	def process_odom_data(self, msg):
		self.current_odom = msg

	def process_image_data(self, msg):
		try:
			image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
		except cv_bridge.CvBridgeError as e:
			print(e)
			return

		# if pose difference is greater than ...
		# store the image



if __name__ == "__main__":

	rospy.init_node("miro_integrate_odom")
	integrator = miro_integrate_odom()
	rospy.spin()
