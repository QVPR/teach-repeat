#!/usr/bin/env python

import rospy
import yaml
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from std_msgs.msg import Bool, String
import message_filters

from teach_repeat.msg import CompressedImageSynchronised
from teach_repeat import image_processing
class miro_camera_calibrator:

	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):	
		self.left_cal_file = rospy.get_param('~calibration_file_left', None)
		self.right_cal_file = rospy.get_param('~calibration_file_right', None)

		if self.left_cal_file is not None:
			with open(self.left_cal_file,'r') as f:
				self.cam_left_calibration = image_processing.yaml_to_camera_info(yaml.load(f.read(), Loader=yaml.SafeLoader))
		else:
			rospy.loginfo('[Image Stamper] no calibration file for left camera specified. Assuming not calibrated')
			self.cam_left_calibration = CameraInfo()
		
		if self.right_cal_file is not None:
			with open(self.right_cal_file,'r') as f:
				self.cam_right_calibration = image_processing.yaml_to_camera_info(yaml.load(f.read(), Loader=yaml.SafeLoader))
		else:
			rospy.loginfo('[Image Stamper] no calibration file for right camera specified. Assuming not calibrated')
			self.cam_right_calibration = CameraInfo()

	def setup_publishers(self):	
		self.pub_image_left = rospy.Publisher("/miro/sensors/cam/left/image_raw", Image, queue_size=0)
		self.pub_image_right = rospy.Publisher("/miro/sensors/cam/right/image_raw", Image, queue_size=0)
		self.pub_image_rect = rospy.Publisher("/miro/sensors/cam/image_rect", Image, queue_size=0)
		self.pub_info_left = rospy.Publisher("/miro/sensors/cam/left/camera_info", CameraInfo, queue_size=0)
		self.pub_info_right = rospy.Publisher("/miro/sensors/cam/right/camera_info", CameraInfo, queue_size=0)

	def setup_subscribers(self):
		# subscribe to the images from both cameras
		self.sub_images = rospy.Subscriber('/miro/sensors/cam/both/compressed', CompressedImageSynchronised, self.process_image_data, queue_size=1, buff_size=2**22)
		self.set_camera_info_left = rospy.Service("/miro/sensors/cam/left/set_camera_info", SetCameraInfo, self.set_camera_info_left)
		self.set_camera_info_right = rospy.Service("/miro/sensors/cam/right/set_camera_info", SetCameraInfo, self.set_camera_info_right)

	def process_image_data(self, msg):
		image_left = image_processing.compressed_msg_to_image(msg.left)
		msg_left = image_processing.image_to_msg(image_left, 'bgr8')
		msg_left.header = msg.left.header
		self.pub_image_left.publish(msg_left)
		info_left = self.cam_left_calibration
		info_left.header = msg.left.header
		self.pub_info_left.publish(info_left)

		image_right = image_processing.compressed_msg_to_image(msg.right)
		msg_right = image_processing.image_to_msg(image_right, 'bgr8')
		msg_right.header = msg.right.header
		self.pub_image_right.publish(msg_right)
		info_right = self.cam_right_calibration
		info_right.header = msg.right.header
		self.pub_info_right.publish(info_right)

		img_stiched = image_processing.rectify_stitch_stereo_image(image_left, image_right, self.cam_left_calibration, self.cam_right_calibration)
		img_stiched_msg = image_processing.image_to_msg(np.uint8(img_stiched), 'mono8')
		img_stiched_msg.header = msg.left.header
		img_stiched_msg.header.frame_id = 'cam'
		self.pub_image_rect.publish(img_stiched_msg)

	def set_camera_info_left(self, srv):
		srv.camera_info.header.frame_id = "caml"
		self.cam_left_calibration = srv.camera_info
		with open(self.left_cal_file,'w') as f:
			f.write(yaml.dump(image_processing.camera_info_to_yaml(srv.camera_info)))
		response = SetCameraInfoResponse()
		response.success = True
		response.status_message = ""
		return response

	def set_camera_info_right(self, srv):
		srv.camera_info.header.frame_id = "camr"
		self.cam_right_calibration = srv.camera_info
		with open(self.right_cal_file,'w') as f:
			f.write(yaml.dump(image_processing.camera_info_to_yaml(srv.camera_info)))
		response = SetCameraInfoResponse()
		response.success = True
		response.status_message = ""
		return response


if __name__ == "__main__":
	rospy.init_node("miro_camera_calibrator")
	calibrator = miro_camera_calibrator()
	rospy.spin()
