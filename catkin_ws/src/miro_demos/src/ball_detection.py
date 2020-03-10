#!/usr/bin/python

import rospy
from sensor_msgs.msg import CompressedImage, Image, JointState
from geometry_msgs.msg import TwistStamped

import cv2
import cv_bridge
import numpy as np
import math

import miro2 as miro

class miro_ball_detection:

	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		# subscribe to the image from the camera
		
		self.sub_image = rospy.Subscriber("/miro/sensors/caml/compressed", CompressedImage, self.got_image_data, queue_size=1)
		# publish the processed image
		self.pub_image = rospy.Publisher("/miro/filtered_image", Image, queue_size=0)

		# publish the desired joint states of Miro (so it looks downwards to view the ball)
		self.pub_joints = rospy.Publisher("/miro/control/kinematic_joints", JointState, queue_size=0)
		self.joint_states = JointState()
		self.joint_states.name = ['tilt','lift','yaw','pitch']
		self.joint_states.position = [0.0, miro.constants.LIFT_RAD_MAX, 0.0, miro.constants.PITCH_RAD_MAX]

		# publish motor commands
		self.pub_cmd_vel = rospy.Publisher("/miro/control/cmd_vel", TwistStamped, queue_size=0)
		

	def got_image_data(self, msg):
		try:
			image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
		except cv_bridge.CvBridgeError as e:
			print(e)
			return

		image = self.filter_image(image)
		
		try:
			self.pub_image.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
		except cv_bridge.CvBridgeError as e:
			print(e)

	def filter_image(self, image):
		image = np.float32(image)

		# gamma correction
		image = image * 1/255
		cv2.pow(image, 2.2, image)

		blue, green, red = cv2.split(image)
		combined = blue + green + red

		# chromacity
		red = cv2.divide(red, combined)
		green = cv2.divide(green, combined)
		blue = cv2.divide(blue, combined)

		_, redT = cv2.threshold(red, 180.0/255.0, 1.0, cv2.THRESH_BINARY)
		_, blueT = cv2.threshold(blue, 30.0/255.0, 1.0, cv2.THRESH_BINARY)
		mask = redT * blueT

		mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,np.ones((5,5),np.float))
		mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,np.ones((3,3),np.float))

		chroma = cv2.merge([blue*mask, green*mask, red*mask])

		points = np.argwhere(mask == 1.0)[:,::-1] # need to reverse x and y
		medXY = np.uint32(np.median(points,0))

		cv2.line(image, (medXY[0]-5,medXY[1]), (medXY[0]+5,medXY[1]), (1,0,0))
		cv2.line(image, (medXY[0],medXY[1]-5), (medXY[0],medXY[1]+5), (1,0,0))

		turn_signal = TwistStamped()
		turn_signal.header.stamp = rospy.Time.now()
		if medXY[0] < (0.7 * image.shape[1]):
			turn_signal.twist.angular.z = 1.0
		elif medXY[0] > (0.9 * image.shape[1]):
			turn_signal.twist.angular.z = -1.0
		self.pub_cmd_vel.publish(turn_signal)

		return np.uint8(image * 255)

	def publish_joint_state(self):
		self.joint_states.header.stamp = rospy.Time.now()
		self.pub_joints.publish(self.joint_states)




if __name__ == "__main__":
	# intialise this program as a ROS node
	rospy.init_node("miro_ball_detection")
	# start the demo program
	demo = miro_ball_detection()
	# publish the desired joint states at 50 Hz
	rate = rospy.Rate(50)
	while not rospy.is_shutdown():
		demo.publish_joint_state()
		rate.sleep()
