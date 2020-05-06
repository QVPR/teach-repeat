#!/usr/bin/python

import rospy
import numpy as np
import cv2
import os
import math
import json
from rospy_message_converter import message_converter
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, UInt32
import tf_conversions

import image_processing
from miro_onboard.msg import CompressedImageSynchronised
from miro_teach_repeat.msg import Goal
from miro_teach_repeat.srv import ImageMatch, ImageMatchRequest

def read_file(filename):
	with open(filename, 'r') as f:
		data = f.read()
		return data

def load_poses(pose_files):
	return [message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',json.loads(read_file(f))) for f in pose_files]

class miro_localiser:
	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):
		self.ready = not rospy.get_param('/wait_for_ready', False)

		# Odom
		self.load_dir = os.path.expanduser(rospy.get_param('miro_data_load_dir', '~/miro/data'))
		if self.load_dir[-1] != '/':
			self.load_dir += '/'

		pose_files = [self.load_dir+f for f in os.listdir(self.load_dir) if f[-9:] == '_pose.txt']
		pose_files.sort()

		self.poses = load_poses(pose_files)
		self.goal_index = 0
		self.goal = self.poses[self.goal_index]

		self.stop_at_end = rospy.get_param('~stop_at_end', True)

		# Image
		self.resize = image_processing.make_size(height=rospy.get_param('/image_resize_height', None), width=rospy.get_param('/image_resize_width', None))
		if self.resize[0] is None and self.resize[1] is None:
			self.resize = None

		self.image_offset_gain = rospy.get_param('~image_offset_gain', 2.3)
		
		self.last_image = None
		self.first_img_seq = 0

		self.save_dir = os.path.expanduser(rospy.get_param('/miro_data_save_dir','~/miro/data/follow-straight_tests/5'))
		if self.save_dir[-1] != '/':
			self.save_dir += '/'
		if not os.path.isdir(self.save_dir):
			os.makedirs(self.save_dir)
		if not os.path.isdir(self.save_dir+'full/'):
			os.makedirs(self.save_dir+'full/')
		if not os.path.isdir(self.save_dir+'norm/'):
			os.makedirs(self.save_dir+'norm/')
		
	def setup_publishers(self):	
		self.goal_pub = rospy.Publisher('/miro/control/goal', Goal, queue_size=1)

		rospy.wait_for_service('/miro/match_image')
		self.match_image = rospy.ServiceProxy('/miro/match_image', ImageMatch, persistent=True)

	def setup_subscribers(self):
		if not self.ready:
			self.sub_ready = rospy.Subscriber("/miro/ready", Bool, self.on_ready, queue_size=1)
		self.sub_odom = rospy.Subscriber("/miro/sensors/odom/integrated", Odometry, self.process_odom_data, queue_size=1)
		self.sub_images = rospy.Subscriber('/miro/sensors/cam/both/compressed', CompressedImageSynchronised, self.process_image_data, queue_size=1, buff_size=2**22)

	def on_ready(self, msg):
		if msg.data:
			if not self.ready:
				localiser.publish_goal(self.poses[0], 0.1, False)
			self.ready = True

	def process_odom_data(self, msg):
		if self.ready and self.last_image is not None:
			current_frame_odom = tf_conversions.fromMsg(msg.pose.pose)
			current_goal_frame_odom = tf_conversions.fromMsg(self.goal)
			old_goal_frame_world = tf_conversions.fromMsg(self.poses[self.goal_index])

			delta_frame = current_frame_odom.Inverse() * current_goal_frame_odom

			if delta_frame.p.Norm() < 0.10:
				old_goal_index = self.goal_index
				self.goal_index += 1
				if self.goal_index == len(self.poses):
					if self.stop_at_end:
						self.goal_index = len(self.poses)-1
						return
					else:
						self.goal_index = 0 # repeat the path (loop)

				image_theta_offset = self.calculate_image_pose_offset(old_goal_index)
				new_goal_frame_world = tf_conversions.fromMsg(self.poses[self.goal_index])

				known_goal_offset = current_goal_frame_odom.Inverse() * current_frame_odom
				# these parameters were estimated from scripts/rotation_displacement_test.py
				expected_pixel_offset = 0.65*math.degrees(known_goal_offset.M.GetRPY()[2]) + 15*known_goal_offset.p.y()
				expected_theta_offset = self.image_offset_gain * float(expected_pixel_offset) / self.last_image.shape[1]

				# old target -> new target in frame of old target (x = forwards)
				goal_offset = old_goal_frame_world.Inverse() * new_goal_frame_world
				# rotate to odom frame (x = positive odom) [same frame as current_goal]
				goal_offset.p = tf_conversions.Rotation.RotZ(current_goal_frame_odom.M.GetRPY()[2]) * goal_offset.p
				# rotate the goal offset by the rotational correction
				goal_offset_corrected = tf_conversions.Frame(tf_conversions.Rotation.RotZ(image_theta_offset - expected_theta_offset)) * goal_offset
				# add the corrected offset to the current goal
				new_goal_odom = tf_conversions.Frame(goal_offset_corrected.M * current_goal_frame_odom.M, goal_offset_corrected.p + current_goal_frame_odom.p)

				self.goal = tf_conversions.toMsg(new_goal_odom)
				if self.goal_index == len(self.poses)-1 and self.stop_at_end:
					self.publish_goal(self.goal, 0.0, True)
				else:
					self.publish_goal(self.goal, 0.1, False)

	def process_image_data(self, msg):
		if self.ready:
			n = msg.left.header.seq
			if self.last_image is None:
				self.first_img_seq = n
			n -= self.first_img_seq

			full_image = image_processing.stitch_stereo_image_message(msg.left, msg.right, compressed=True)
			normalised_image = image_processing.patch_normalise_image(full_image, (9,9), resize=self.resize)
			
			cv2.imwrite(self.save_dir+('full/%06d.png' % n), full_image)
			cv2.imwrite(self.save_dir+('norm/%06d.png' % n), np.uint8(255.0 * (1 + normalised_image) / 2.0))

			self.last_image = normalised_image

	def publish_goal(self, pose, lookahead_distance=0.0, stop_at_goal=False):
		goal = Goal()
		goal.pose.header.stamp = rospy.Time.now()
		goal.pose.header.frame_id = "odom"
		theta = tf_conversions.fromMsg(pose).M.GetRPY()[2]
		goal.pose.pose.position.x = pose.position.x + lookahead_distance * math.cos(theta)
		goal.pose.pose.position.y = pose.position.y + lookahead_distance * math.sin(theta)
		goal.pose.pose.orientation = pose.orientation
		goal.stop_at_goal.data = stop_at_goal
		self.goal_pub.publish(goal)

	def calculate_image_pose_offset(self, image_to_search_index):
		if self.last_image is not None:
			matchRequest = ImageMatchRequest(image_processing.image_to_msg(self.last_image), UInt32(image_to_search_index))
			image_offset = self.match_image(matchRequest).pixelOffset.data

			# positive image offset: query image is shifted left from reference image
			# this means we have done a right (negative turn) which we should correct with a positive turn
			# positive offset -> positive turn, gain = positive
			# (normalise the pixel offset by the width of the image)
			return self.image_offset_gain * float(image_offset) / self.last_image.shape[1]
		else:
			raise RuntimeError('Localiser: tried to localise before image data is received!')

if __name__ == "__main__":
	rospy.init_node("miro_localiser")
	localiser = miro_localiser()
	if localiser.ready:
		localiser.publish_goal(localiser.poses[0])
	rospy.spin()