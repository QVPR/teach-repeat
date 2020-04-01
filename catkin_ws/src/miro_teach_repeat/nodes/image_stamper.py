#!/usr/bin/python

import rospy
from sensor_msgs.msg import CompressedImage, CameraInfo
import cv_bridge
import yaml

bridge = cv_bridge.CvBridge()


def yaml_to_camera_info(yaml_data):
	camera_info = CameraInfo()
	camera_info.header.frame_id = yaml_data['camera_name']
	camera_info.D = yaml_data['distortion_coefficients']['data']
	camera_info.K = yaml_data['camera_matrix']['data']
	camera_info.P = yaml_data['projection_matrix']['data']
	camera_info.R = yaml_data['rectification_matrix']['data']
	camera_info.distortion_model = yaml_data['distortion_model']
	camera_info.height = yaml_data['image_height']
	camera_info.width = yaml_data['image_width']
	return camera_info

def camera_info_to_yaml(camera_info):
	yaml_data = {}
	yaml_data['camera_name'] = camera_info.header.frame_id
	yaml_data['distortion_coefficients'] = {'data':camera_info.D,'cols':5,'rows':1}
	yaml_data['camera_matrix'] = {'data':camera_info.K,'cols':3,'rows':3}
	yaml_data['projection_matrix'] = {'data':camera_info.P,'cols':4,'rows':3}
	yaml_data['rectification_matrix'] = {'data':camera_info.R,'cols':3,'rows':3}
	yaml_data['distortion_model'] = camera_info.distortion_model
	yaml_data['image_height'] = camera_info.height
	yaml_data['image_width'] = camera_info.width
	return yaml_data

class miro_image_stamper:

	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()		

	def setup_parameters(self):
		self.left_cal_file = rospy.get_param('~calibration_file_left', None)
		self.right_cal_file = rospy.get_param('~calibration_file_right', None)

		if self.left_cal_file is not None:
			with open(self.left_cal_file,'r') as f:
				self.cam_left_calibration = yaml_to_camera_info(yaml.load(f.read()))
		else:
			rospy.loginfo('[Image Stamper] no calibration file for left camera specified. Assuming not calibrated')
			self.cam_left_calibration = CameraInfo()
		
		if self.right_cal_file is not None:
			with open(self.right_cal_file,'r') as f:
				self.cam_right_calibration = yaml_to_camera_info(yaml.load(f.read()))
		else:
			rospy.loginfo('[Image Stamper] no calibration file for right camera specified. Assuming not calibrated')
			self.cam_right_calibration = CameraInfo()
		
	def setup_publishers(self):
		# publish stamped image
		self.pub_image_left = rospy.Publisher("/miro/sensors/cam_stamped/left/compressed", CompressedImage, queue_size=0)
		self.pub_image_right = rospy.Publisher("/miro/sensors/cam_stamped/right/compressed", CompressedImage, queue_size=0)
		self.pub_info_left = rospy.Publisher("/miro/sensors/cam_stamped/left/camera_info", CameraInfo, queue_size=0)
		self.pub_info_right = rospy.Publisher("/miro/sensors/cam_stamped/right/camera_info", CameraInfo, queue_size=0)

	def setup_subscribers(self):
		# subscribe to image
		self.sub_image_left = rospy.Subscriber("/miro/sensors/caml/compressed", CompressedImage, self.process_image_data_left, queue_size=1)
		self.sub_image_right = rospy.Subscriber("/miro/sensors/camr/compressed", CompressedImage, self.process_image_data_right, queue_size=1)

	def process_image_data_left(self, msg):
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "caml"
		info_left = self.cam_left_calibration
		info_left.header = msg.header

		if len(msg.data) == 0:
			rospy.logwarn("[Image Stamper] received empty compressed image - dropping message")
			return
		self.pub_image_left.publish(msg)
		self.pub_info_left.publish(info_left)

	def process_image_data_right(self, msg):
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "camr"

		info_right = self.cam_right_calibration
		info_right.header = msg.header

		if len(msg.data) == 0:
			rospy.logwarn("[Image Stamper] received empty compressed image - dropping message")
			return
		self.pub_image_right.publish(msg)
		self.pub_info_right.publish(info_right)

if __name__ == "__main__":

	rospy.init_node("miro_image_stamper")
	stamper = miro_image_stamper()
	rospy.spin()

