#!/usr/bin/python

import rospy
import yaml
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo
from std_msgs.msg import Bool, String
import message_filters

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

class miro_image_sync:

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
		self.pub_image_left = rospy.Publisher("/miro/sensors/cam_sync/left/image_raw", Image, queue_size=0)
		self.pub_image_right = rospy.Publisher("/miro/sensors/cam_sync/right/image_raw", Image, queue_size=0)
		self.pub_info_left = rospy.Publisher("/miro/sensors/cam_sync/left/camera_info", CameraInfo, queue_size=0)
		self.pub_info_right = rospy.Publisher("/miro/sensors/cam_sync/right/camera_info", CameraInfo, queue_size=0)

	def setup_subscribers(self):
		# subscribe to the images from both cameras
		self.sub_image_left = message_filters.Subscriber("/miro/sensors/cam_stamped/left/image", Image, queue_size=1)
		self.sub_image_right = message_filters.Subscriber("/miro/sensors/cam_stamped/right/image", Image, queue_size=1)
		self.sub_images = message_filters.ApproximateTimeSynchronizer((self.sub_image_left, self.sub_image_right), 5, 1.0/30.0)
		self.sub_images.registerCallback(self.process_image_data)
		self.set_camera_info_left = rospy.Service("/miro/sensors/cam_sync/left/set_camera_info", SetCameraInfo, self.set_camera_info_left)
		self.set_camera_info_right = rospy.Service("/miro/sensors/cam_sync/right/set_camera_info", SetCameraInfo, self.set_camera_info_right)

	def process_image_data(self, msg_left, msg_right):
		average_time = rospy.Time.from_sec((msg_left.header.stamp.to_sec() + msg_right.header.stamp.to_sec()) / 2)

		msg_left.header.stamp = average_time
		self.pub_image_left.publish(msg_left)
		msg_right.header.stamp = average_time
		self.pub_image_right.publish(msg_right)

		info_left = self.cam_left_calibration
		info_left.header = msg_left.header
		self.pub_info_left.publish(info_left)

		info_right = self.cam_right_calibration
		info_right.header = msg_right.header
		self.pub_info_right.publish(info_right)

	def set_camera_info_left(self, srv):
		srv.camera_info.header.frame_id = "caml"
		self.cam_left_calibration = srv.camera_info
		with open(self.left_cal_file,'w') as f:
			f.write(yaml.dump(camera_info_to_yaml(srv.camera_info)))
		srv.success = True
		srv.status_messages = ""
		return srv

	def set_camera_info_right(self, srv):
		srv.camera_info.header.frame_id = "camr"
		self.cam_right_calibration = srv.camera_info
		with open(self.right_cal_file,'w') as f:
			f.write(yaml.dump(camera_info_to_yaml(srv.camera_info)))
		srv.success = True
		srv.status_messages = ""
		return srv



if __name__ == "__main__":

	rospy.init_node("miro_image_sync")
	syncer = miro_image_sync()
	rospy.spin()

