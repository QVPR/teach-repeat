#!/usr/bin/python

import rospy
from sensor_msgs.msg import CompressedImage, Image, JointState
from geometry_msgs.msg import TwistStamped
import message_filters

import cv2
import cv_bridge
import numpy as np
import math

import miro2 as miro

class miro_ball_detection:

	def __init__(self):
		# subscribe to the images from both cameras
		self.sub_image_left = message_filters.Subscriber("/miro/sensors/caml/compressed", CompressedImage, queue_size=1)
		self.sub_image_right = message_filters.Subscriber("/miro/sensors/camr/compressed", CompressedImage, queue_size=1)
		self.sub_images = message_filters.ApproximateTimeSynchronizer((self.sub_image_left, self.sub_image_right), 5, 1.0/15.0)
		self.sub_images.registerCallback(self.process_image_data)
		# needed for converting the images
		self.bridge = cv_bridge.CvBridge()
		# publish the processed image
		self.pub_image = rospy.Publisher("/miro/filtered_image", Image, queue_size=0)

		# publish the desired joint states of Miro (so it looks downwards to view the ball)
		self.pub_joints = rospy.Publisher("/miro/control/kinematic_joints", JointState, queue_size=0)
		self.joint_states = JointState()
		self.joint_states.name = ['tilt','lift','yaw','pitch']
		self.joint_states.position = [0.0, miro.constants.LIFT_RAD_MAX, 0.0, miro.constants.PITCH_RAD_MAX]

		# publish motor commands
		self.pub_cmd_vel = rospy.Publisher("/miro/control/cmd_vel", TwistStamped, queue_size=0)

	def process_image_data(self, msgLeft, msgRight):
		try:
			image_left = self.bridge.compressed_imgmsg_to_cv2(msgLeft, "bgr8")
			image_right = self.bridge.compressed_imgmsg_to_cv2(msgRight, "bgr8")
		except cv_bridge.CvBridgeError as e:
			print(e)
			return

		ballX_left, ballY_left = self.detect_ball_in_image(image_left)
		ballX_right, ballY_right = self.detect_ball_in_image(image_right)

		# draw the medians in the image
		if ballX_left is not None:
			cv2.line(image_left, (ballX_left-15,ballY_left), (ballX_left+15,ballY_left), (255,0,0))
			cv2.line(image_left, (ballX_left,ballY_left-15), (ballX_left,ballY_left+15), (255,0,0))
		if ballX_right is not None:
			cv2.line(image_right, (ballX_right-15,ballY_right), (ballX_right+15,ballY_right), (255,0,0))
			cv2.line(image_right, (ballX_right,ballY_right-15), (ballX_right,ballY_right+15), (255,0,0))

		# send motor commands to turn to the ball
		# if ball to the left turn left, vice versa, if in the middle and far away drive straight
		turn_signal = TwistStamped()
		turn_signal.header.stamp = rospy.Time.now()
		if ballX_left is not None and ballX_left < (0.7 * image_left.shape[1]):
			turn_signal.twist.angular.z = 1.0
		elif ballX_right is not None and ballX_right > (0.3 * image_right.shape[1]):
			turn_signal.twist.angular.z = -1.0
		elif (ballY_left is not None and ballY_left < (0.6 * image_left.shape[0])) or (ballY_right is not None and ballY_right < (0.6 * image_right.shape[0])):
			turn_signal.twist.linear.x = 0.1
		self.pub_cmd_vel.publish(turn_signal)

		image = np.concatenate((image_left, image_right), axis=1)
		
		try:
			self.pub_image.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
		except cv_bridge.CvBridgeError as e:
			print(e)

	def detect_ball_in_image(self, image):
		# convert image from 8 bit uint (0->255) to 32 bit float (0.0 -> 1.0)
		image = np.float32(image)
		image = image / 255.0

		# gamma correction
		image = cv2.pow(image, 2.2)

		# split into colour channels
		blue, green, red = cv2.split(image)

		# chromacity
		combined = blue + green + red
		red = cv2.divide(red, combined)
		green = cv2.divide(green, combined)
		blue = cv2.divide(blue, combined)

		# threshold red and blue channels
		_, redT = cv2.threshold(red, 180.0/255.0, 1.0, cv2.THRESH_BINARY)
		_, blueT = cv2.threshold(blue, 30.0/255.0, 1.0, cv2.THRESH_BINARY)
		# combine the masks
		mask = redT * blueT

		# fill in holes and remove noise
		mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,np.ones((5,5),np.float))
		mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,np.ones((3,3),np.float))

		# find the median location of points in the mask
		points = np.argwhere(mask == 1.0)[:,::-1] # need to reverse x and y
		# if we have enough points - find the ball
		if len(points) > 100:
			medXY = np.uint32(np.median(points,0))
			return (medXY[0], medXY[1])
		else:
			return (None, None)

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
