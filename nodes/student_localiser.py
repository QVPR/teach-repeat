#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import os
import math
import json
import datetime
import enum
import threading
import time
from rospy_message_converter import message_converter
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt32
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import PoseStamped, Pose
import tf_conversions
import tf
import tf2_ros

import teach_repeat.image_processing as image_processing
from teach_repeat.msg import Goal
from teach_repeat.srv import ImageMatch, ImageMatchRequest

FIELD_OF_VIEW_DEG = 2*60.6 + 2*27.0
IMAGE_WIDTH = 115.0 # px
CORR_SUB_SAMPLING = 1

GOAL_DISTANCE_SPACING = 0.2 # m
LOOKAHEAD_DISTANCE_RATIO = 0.65 # x GOAL_DISTANCE_SPACING
TURNING_TARGET_RANGE_DISTANCE_RATIO = 0.5 # x GOAL_DISTANCE_SPACING
# LOOKAHEAD_DISTANCE_RATIO > TURNING_TARGET_RANGE_DISTANCE_RATIO (otherwise stops and turns on spot when arriving at goal)
GOAL_THETA_TOLERANCE = 5 #deg

class GOAL_STATE(enum.Enum):
	normal_goal = 0
	finished = 1
	restart = 2

# Do not need for multi-robot system
#def read_file(filename):
#	with open(filename, 'r') as f:
#		data = f.read()
#		return data
#
#def load_poses(pose_files):
#	return [message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',json.loads(read_file(f))) for f in pose_files]

def wrapToPi(x):
	'''wrap angle to between +pi and -pi'''
	return ((x + math.pi) % (2*math.pi)) - math.pi

def normalise_quaternion(q):
	'''normalise a ROS geometry_msgs/Quaternion so its magnitude is 1'''
	norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
	q.x /= norm
	q.y /= norm
	q.z /= norm
	q.w /= norm

def px_to_deg(px):
	'''convert image pixel offset to rotational offset (deg)'''
	return px * FIELD_OF_VIEW_DEG / IMAGE_WIDTH / CORR_SUB_SAMPLING

def deg_to_px(deg):
	'''convert rotational offset (deg) to image pixels'''
	return deg * IMAGE_WIDTH * CORR_SUB_SAMPLING / FIELD_OF_VIEW_DEG

def px_to_rad(px):
	'''convert image pixel offset to rotational offset (rad)'''
	return px * math.radians(FIELD_OF_VIEW_DEG) / IMAGE_WIDTH / CORR_SUB_SAMPLING

def rad_to_px(rad):
	'''convert rotational offset (rad) to image pixels'''
	return rad * IMAGE_WIDTH * CORR_SUB_SAMPLING / math.radians(FIELD_OF_VIEW_DEG)

def delta_frame_in_bounds(delta_frame):
	'''determine whether the difference between current location and goal is within bounds'''
	distance = delta_frame.p.Norm()
	angle = wrapToPi(delta_frame.M.GetRPY()[2])
	if distance < (LOOKAHEAD_DISTANCE_RATIO * GOAL_DISTANCE_SPACING):
		if abs(angle) < math.radians(GOAL_THETA_TOLERANCE):
			return True
		else:
			pass
			print('within goal distance, but theta offset not within tolerance = %.2f degrees [distance = %.3f]' % (math.degrees(angle), distance))
	return False

def get_expected_px_offset(offset_frame):
	'''get the expect pixel offset given the robot's offset from the goal'''
	return -rad_to_px(offset_frame.M.GetRPY()[2])

def get_corrected_goal_offset(goal1, goal2, rotation_correction, correction_length):
	'''apply rotation and length corrections to the offset from goal1 to goal2'''
	goal_offset = goal1.Inverse() * goal2
	goal_offset = tf_conversions.Frame(tf_conversions.Rotation.RotZ(rotation_correction)) * goal_offset
	goal_offset.p *= correction_length
	return goal_offset

def is_turning_goal(goal_frame, next_goal_frame):
	'''calculate whether the offset from goal -> next goal is requires turning on the spot'''
	diff = goal_frame.Inverse() * next_goal_frame
	dist = diff.p.Norm()
	return dist < (TURNING_TARGET_RANGE_DISTANCE_RATIO * GOAL_DISTANCE_SPACING)

class teach_repeat_localiser:
	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):
		# Teach Repeat Params
		self.rotation_correction_gain = rospy.get_param('~rotation_correction_gain', 0.01)
		self.path_correction_gain = rospy.get_param('~path_correction_gain', 0.01)

		# Wait for ready
		self.ready = not rospy.get_param('/wait_for_ready', False)
		self.global_localisation_init = rospy.get_param('~global_localisation_init', False)
		self.min_init_correlation = rospy.get_param('~min_init_correlation', 0.0)
		self.running = False
		self.mutex = threading.Lock()

		# Odom
		self.load_dir = os.path.expanduser(rospy.get_param('/data_load_dir', '~/miro/data'))
		if self.load_dir[-1] != '/':
			self.load_dir += '/'
		self.sum_theta_correction = 0.0
		self.sum_path_correction = 0.0
		self.stop_at_end = rospy.get_param('~stop_at_end', True)
		self.discrete_correction = rospy.get_param('~discrete-correction', False)

		# For Multi-Robot Student Delay
		self.frames_delay = rospy.get_param('~frames_delay', 10)

		# Load pose data
		# Do Not need the download
		#pose_files = [self.load_dir+f for f in os.listdir(self.load_dir) if f[-9:] == '_pose.txt']
		#pose_files.sort()
		self.poses = []
		self.goal_index = 0
		self.goal = tf_conversions.toMsg(tf_conversions.Frame())
		self.last_goal = tf_conversions.toMsg(tf_conversions.Frame())
		self.goal_plus_lookahead = tf_conversions.toMsg(tf_conversions.Frame())
		self.last_odom_pose = None
		self.zero_odom_offset = None
		global GOAL_DISTANCE_SPACING, LOOKAHEAD_DISTANCE_RATIO, TURNING_TARGET_RANGE_DISTANCE_RATIO, GOAL_THETA_TOLERANCE
		GOAL_DISTANCE_SPACING = rospy.get_param('/goal_pose_separation', 0.2)
		LOOKAHEAD_DISTANCE_RATIO = rospy.get_param('/lookahead_distance_ratio', 0.65)
		TURNING_TARGET_RANGE_DISTANCE_RATIO = rospy.get_param('/turning_target_range_distance_ratio', 0.5)
		GOAL_THETA_TOLERANCE = rospy.get_param('/goal_theta_tolerance', 5)

		# Image
		self.resize = image_processing.make_size(height=rospy.get_param('/image_resize_height', None), width=rospy.get_param('/image_resize_width', None))
		if self.resize[1] is not None:
			global IMAGE_WIDTH
			IMAGE_WIDTH = self.resize[1]
		if self.resize[0] is None and self.resize[1] is None:
			self.resize = None
		self.last_image = None
		self.patch_size = image_processing.parse_patch_size_parameter(rospy.get_param('/patch_size', (9,9)))
		global CORR_SUB_SAMPLING, FIELD_OF_VIEW_DEG
		self.image_recognition_threshold = rospy.get_param('/image_recognition_threshold', 0.1)
		CORR_SUB_SAMPLING = rospy.get_param('/image_subsampling', 1)
		FIELD_OF_VIEW_DEG = rospy.get_param('/image_field_of_view_width_deg', 2*60.6 + 2*27.0)
		FIELD_OF_VIEW_RAD = math.radians(FIELD_OF_VIEW_DEG)
		self.search_range = rospy.get_param('~search-range', 1)

		# data saving
		self.save_dir = os.path.expanduser(rospy.get_param('/data_save_dir', '~/miro/data/follow-straight_tests/5'))
		self.save_full_res_images = rospy.get_param('/save_full_res_images', True)
		self.save_full_res_images_at_goal = rospy.get_param('/save_full_res_images_at_goal', True)
		self.last_full_res_image = None
		self.save_gt_data = rospy.get_param('/save_gt_data', False)
		self.publish_gt_goals = rospy.get_param('/publish_gt_goals', True)
		if self.save_dir[-1] != '/':
			self.save_dir += '/'
		if not os.path.isdir(self.save_dir):
			os.makedirs(self.save_dir)
		if not os.path.isdir(self.save_dir+'norm/'):
			os.makedirs(self.save_dir+'norm/')
		if not os.path.isdir(self.save_dir+'pose/'):
			os.makedirs(self.save_dir+'pose/')
		if not os.path.isdir(self.save_dir+'offset/'):
			os.makedirs(self.save_dir+'offset/')
		if not os.path.isdir(self.save_dir+'correction/'):
			os.makedirs(self.save_dir+'correction/')
		if self.save_full_res_images or self.save_full_res_images_at_goal:
			if not os.path.isdir(self.save_dir+'full/'):
				os.makedirs(self.save_dir+'full/')
		self.goal_number = 0

		self.save_params()


	def setup_publishers(self):
		self.goal_pub = rospy.Publisher('goal', Goal, queue_size=1)
		if self.publish_gt_goals:
			self.goal_pose_pub = rospy.Publisher('goal_pose', PoseStamped, queue_size=1)
		rospy.wait_for_service('match_image')
		self.match_image = rospy.ServiceProxy('match_image', ImageMatch, persistent=True)

	def setup_subscribers(self):
		if not self.ready:
			self.srv_ready = rospy.Service('ready_localiser', Trigger, self.on_ready)
		self.sub_odom = rospy.Subscriber("odom", Odometry, self.process_odom_data, queue_size=1)
		self.sub_images = rospy.Subscriber('image', Image, self.process_image_data, queue_size=1, buff_size=2**22)
		self.tfBuffer = tf2_ros.Buffer()
		self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

		# For Multi-Robot System
		self.sub_teacher_pose = rospy.Subscriber("teacher_pose", Pose, self.append_teacher_pose, queue_size=1)

	def append_teacher_pose(self, msg):
		'''Append new teacher pose to the student list when ever it arrived'''
		self.poses.append(msg)

	def save_params(self):
		params = {
			'load_dir' : self.load_dir,
			'rotation_correction_gain' : self.rotation_correction_gain,
			'path_correction_gain' : self.path_correction_gain,
			'wait_for_ready' : not self.ready,
			'global_localisation_init' : self.global_localisation_init,
			'resize': self.resize,
			'patch_size': self.patch_size,
			'stop_at_end': self.stop_at_end,
			'discrete_correction': self.discrete_correction,
			'goal_distance_spacing': GOAL_DISTANCE_SPACING,
			'lookahead_distance_ratio': LOOKAHEAD_DISTANCE_RATIO,
			'turning_target_theta_range_distance_ratio': TURNING_TARGET_RANGE_DISTANCE_RATIO,
			'goal_theta_tolerance': GOAL_THETA_TOLERANCE,
			'image_recognition_threshold': self.image_recognition_threshold,
			'image_subsampling': CORR_SUB_SAMPLING,
			'image_field_of_view_width_deg': FIELD_OF_VIEW_DEG,
			'search_range': self.search_range,
			'save_full_res_images': self.save_full_res_images,
			'save_gt_data': self.save_gt_data
		}
		with open(self.save_dir + 'params.txt', 'w') as params_file:
			params_file.write(json.dumps(params))

	def on_ready(self, srv):
		if not self.ready:
			self.start()
			self.ready = True
			return TriggerResponse(success=True)
		else:
			return TriggerResponse(success=False, message="Localiser already started.")

	def start(self):
		while (len(self.poses) < self.frames_delay):
			pass
		if self.global_localisation_init:
			if self.last_odom_pose is None or self.last_image is None:
				print('Global localisation - waiting for first odom and image messages')
				while self.last_odom_pose is None or self.last_image is None:
					time.sleep(0.2) # sleep lets subscribers be processed

			offsets, correlations = self.calculate_image_pose_offset(0, len(self.poses))
			best_match = np.argmax(correlations)
			if best_match == 0 or (best_match == 1 and correlations[0] > correlations[2]):
				goal_pose = tf_conversions.Frame()
				odom_pose = tf_conversions.fromMsg(self.last_odom_pose)
				self.zero_odom_offset = tf_conversions.fromMsg(self.last_odom_pose)

				if self.poses[0].position.x == 0 and self.poses[0].position.y == 0 and tf_conversions.fromMsg(self.poses[0]).M.GetRPY()[2] == 0:
					# the first pose is at 0,0,0 so we ignore it and set the first goal to the next pose
					print('Localiser: starting at goal 1, goal 0 = [0,0,0]')
					self.goal_index = 1
				else:
					self.goal_index = 0
			else:
				if best_match == len(self.poses)-1:
					self.goal_index = len(self.poses)-2
				elif correlations[best_match+1] >= correlations[best_match-1]:
					self.goal_index = best_match
				else:
					self.goal_index = best_match-1
				# goal = offset.Inverse() * odom
				# offset.Inverse() = goal * odom.Inverse()
				# offset = (goal * odom.Inverse()).Inverse()
				goal_pose = tf_conversions.fromMsg(self.poses[self.goal_index-1])
				odom_pose = tf_conversions.fromMsg(self.last_odom_pose)
				self.zero_odom_offset = (goal_pose * odom_pose.Inverse()).Inverse()
			self.last_odom_pose = tf_conversions.toMsg(self.zero_odom_offset.Inverse() * tf_conversions.fromMsg(self.last_odom_pose))
			print('Global localisation - best match at pose %d [correlation = %f]' % (self.goal_index, correlations[best_match]))
			if correlations[best_match] < self.min_init_correlation:
				print('Stop, best match [%.2f] did not exceed minimum correlation ([%.2f])' % (correlations[best_match], self.min_init_correlation))
				return
		else:
			if self.last_odom_pose is None:
				print('Global localisation - waiting for first odom message')
				while self.last_odom_pose is None:
					time.sleep(0.2) # sleep lets subscribers be processed

			goal_pose = tf_conversions.Frame()
			odom_pose = tf_conversions.fromMsg(self.last_odom_pose)
			self.zero_odom_offset = tf_conversions.fromMsg(self.last_odom_pose)

			if self.poses[0].position.x == 0 and self.poses[0].position.y == 0 and tf_conversions.fromMsg(self.poses[0]).M.GetRPY()[2] == 0:
				# the first pose is at 0,0,0 so we ignore it and set the first goal to the next pose
				print('Localiser: starting at goal 1, goal 0 = [0,0,0]')
				self.goal_index = 1
			else:
				self.goal_index = 0

		self.update_goal(tf_conversions.fromMsg(self.poses[self.goal_index]))

		self.running = True

	def update_goal_index(self):
		self.goal_index += 1
		if self.goal_index == len(self.poses):
			if self.stop_at_end:
				self.goal_index = len(self.poses) - 1
				# save image when arriving at the final goal
				if self.goal_number < len(self.poses):
					self.calculate_image_pose_offset(self.goal_index)
					self.goal_number += 1
				return GOAL_STATE.finished
			else:
				self.goal_index = 0 # repeat the path (loop)
				return GOAL_STATE.restart
		return GOAL_STATE.normal_goal

	def save_data_at_goal(self, pose, goal_odom, goal_world, theta_offset, path_offset):
		if self.save_gt_data:
			try:
				trans = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
				trans_as_text = json.dumps(message_converter.convert_ros_message_to_dictionary(trans))
				with open(self.save_dir + ('%06d_map_to_base_link.txt' % self.goal_number), 'w') as pose_file:
					pose_file.write(trans_as_text)
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				print('Could not lookup transform from /map to /base_link')
				pass

		# save full res image at goal
		if self.save_full_res_images_at_goal:
			cv2.imwrite(self.save_dir+('full/goal_%06d.png' % self.goal_number), self.last_full_res_image)

		# save current pose info
		message_as_text = json.dumps(message_converter.convert_ros_message_to_dictionary(pose))
		with open(self.save_dir+('pose/%06d_pose.txt' % self.goal_number), 'w') as pose_file:
			pose_file.write(message_as_text)
		# save current offset
		offset = tf_conversions.toMsg(goal_odom.Inverse() * goal_world)
		message_as_text = json.dumps(message_converter.convert_ros_message_to_dictionary(offset))
		with open(self.save_dir+('offset/%06d_offset.txt' % self.goal_number), 'w') as offset_file:
			offset_file.write(message_as_text)
		# publish offset to tf
		# self.tf_pub.sendTransform((offset.position.x, offset.position.y, offset.position.z), (offset.orientation.x, offset.orientation.y, offset.orientation.z, offset.orientation.w), rospy.Time.now(), 'map', 'odom')
		# publish current corrections
		message_as_text = json.dumps({'theta_offset': theta_offset, 'path_offset': path_offset})
		with open(self.save_dir+('correction/%06d_correction.txt' % self.goal_number), 'w') as correction_file:
			correction_file.write(message_as_text)

	def subtract_odom(self, odom, odom_frame_to_subtract):
		odom_frame = tf_conversions.fromMsg(odom.pose.pose)
		subtracted_odom = odom_frame_to_subtract.Inverse() * odom_frame
		odom.pose.pose = tf_conversions.toMsg(subtracted_odom)
		return odom

	def process_odom_data(self, msg):
		if self.running:
			if self.last_image is not None:
				self.mutex.acquire()
				if self.zero_odom_offset is None:
					self.zero_odom_offset = tf_conversions.fromMsg(msg.pose.pose)
				msg = self.subtract_odom(msg, self.zero_odom_offset)
				self.last_odom_pose = msg.pose.pose

				current_pose_odom = tf_conversions.fromMsg(msg.pose.pose)
				current_goal_frame_odom = tf_conversions.fromMsg(self.goal)
				current_goal_plus_lookahead_frame_odom = tf_conversions.fromMsg(self.goal_plus_lookahead)
				old_goal_frame_world = tf_conversions.fromMsg(self.poses[self.goal_index])

				delta_frame = current_pose_odom.Inverse() * current_goal_plus_lookahead_frame_odom

				if delta_frame_in_bounds(delta_frame):
					if self.discrete_correction:
						rotation_correction, path_correction = self.do_discrete_correction(msg.pose.pose, current_goal_frame_odom, old_goal_frame_world)
						self.make_new_goal(rotation_correction, path_correction)
					else:
						self.make_new_goal()
				self.mutex.release()
		else:
			self.mutex.acquire()
			self.last_odom_pose = msg.pose.pose
			self.mutex.release()

	def process_image_data(self, msg):
		if self.running:
			n = msg.header.seq

			full_image = image_processing.msg_to_image(msg)
			normalised_image = image_processing.patch_normalise_image(full_image, self.patch_size, resize=self.resize)

			if self.save_full_res_images:
				cv2.imwrite(self.save_dir+('full/%06d.png' % n), full_image)
			cv2.imwrite(self.save_dir+('norm/%06d.png' % n), np.uint8(255.0 * (1 + normalised_image) / 2.0))

			self.mutex.acquire()
			self.last_image = normalised_image
			if self.save_full_res_images_at_goal:
				self.last_full_res_image = full_image

			if not self.discrete_correction:
				self.do_continuous_correction()
			self.mutex.release()
		else:
			self.mutex.acquire()
			self.last_image = image_processing.patch_normalise_image(image_processing.msg_to_image(msg), (9,9), resize=self.resize)
			self.mutex.release()

	def make_new_goal(self, rotation_correction=0.0, path_correction=1.0):
		old_goal_index = self.goal_index
		old_goal_frame_world = tf_conversions.fromMsg(self.poses[old_goal_index])
		current_goal_frame_odom = tf_conversions.fromMsg(self.goal)

		state = self.update_goal_index()

		if state == GOAL_STATE.finished:
			print('Localiser stopping. Reached final goal.')
			self.running = False
			return

		if state == GOAL_STATE.restart:
			# don't have a big offset from the end of the path, back to the start
			old_goal_frame_world = tf_conversions.Frame()

		new_goal_frame_world = tf_conversions.fromMsg(self.poses[self.goal_index])
		turning_goal = is_turning_goal(old_goal_frame_world, new_goal_frame_world)
		goal_offset = get_corrected_goal_offset(old_goal_frame_world, new_goal_frame_world, rotation_correction, path_correction)
		new_goal = current_goal_frame_odom * goal_offset

		sum_path_correction_ratio = (GOAL_DISTANCE_SPACING + self.sum_path_correction) / GOAL_DISTANCE_SPACING

		self.update_goal(new_goal, True, turning_goal)
		self.save_data_at_goal(self.last_odom_pose, new_goal, new_goal_frame_world, self.sum_theta_correction, sum_path_correction_ratio)
		self.goal_number += 1
		print('[%d] theta [%f]\tpath [%f]' % (old_goal_index, math.degrees(self.sum_theta_correction), sum_path_correction_ratio))
		if turning_goal:
			print('turning goal:')
		self.sum_theta_correction = 0
		self.sum_path_correction = 0.0

	def update_goal(self, goal_frame, new_goal=True, turning_goal=False):
		if new_goal:
			self.last_goal = self.goal

		self.goal = tf_conversions.toMsg(goal_frame)
		normalise_quaternion(self.goal.orientation)

		# if goal is a turning goal, or the last goal - don't set virtual waypoint ahead
		if turning_goal or (self.goal_index == len(self.poses)-1 and self.stop_at_end):
			self.publish_goal(self.goal, 0.0, True)
		else:
			self.publish_goal(self.goal, (LOOKAHEAD_DISTANCE_RATIO * GOAL_DISTANCE_SPACING), False)

	def publish_goal(self, pose, lookahead_distance=0.0, stop_at_goal=False):
		goal = Goal()
		goal.pose.header.stamp = rospy.Time.now()
		goal.pose.header.frame_id = "odom"

		lookahead = tf_conversions.Frame(tf_conversions.Vector(lookahead_distance, 0, 0))

		# add the offset for navigating to the goal:
		# (offset * odom).Inverse() * goal = odom.Invserse() * pose
		# goal = (offset * odom) * odom.Inverse() * pose
		# goal = offset * pose
		original_pose_frame = tf_conversions.fromMsg(pose)
		pose_frame = self.zero_odom_offset * original_pose_frame
		original_pose_frame_lookahead = original_pose_frame * lookahead
		pose_frame_lookahead = pose_frame * lookahead

		goal.pose.pose.position.x = pose_frame_lookahead.p.x()
		goal.pose.pose.position.y = pose_frame_lookahead.p.y()
		goal.pose.pose.orientation = tf_conversions.toMsg(pose_frame_lookahead).orientation

		goal.stop_at_goal.data = stop_at_goal
		self.goal_pub.publish(goal)
		self.goal_plus_lookahead = tf_conversions.toMsg(original_pose_frame_lookahead)

		if self.publish_gt_goals:
			try:
				trans = self.tfBuffer.lookup_transform('map', 'odom', rospy.Time())
				trans_frame = tf_conversions.Frame(tf_conversions.Rotation(trans.rotation.x,trans.rotation.y,trans.rotation.z,trans.rotation.w),
				tf_conversions.Vector(trans.translation.x,trans.translation.y,trans.translation.z))

				goal_pose = PoseStamped()
				goal_pose.header.stamp = rospy.Time.now()
				goal_pose.header.frame_id = "map"
				goal_pose.pose = tf_conversions.toMsg(trans_frame * pose_frame_lookahead)
				self.goal_pose_pub.publish(goal_pose)
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				print('Could not lookup transform from /map to /odom')
				pass

	def do_continuous_correction(self):
		if self.last_odom_pose is not None and self.goal_index > 0:
			next_goal_world = tf_conversions.fromMsg(self.poses[self.goal_index])
			last_goal_world = tf_conversions.fromMsg(self.poses[self.goal_index-1])
			last_goal_odom = tf_conversions.fromMsg(self.last_goal)
			next_goal_odom = tf_conversions.fromMsg(self.goal)
			current_pose_odom = tf_conversions.fromMsg(self.last_odom_pose)

			next_goal_offset_odom = next_goal_odom.Inverse() * current_pose_odom
			last_goal_offset_odom = last_goal_odom.Inverse() * current_pose_odom
			inter_goal_offset_odom = last_goal_odom.Inverse() * next_goal_odom

			last_goal_distance = last_goal_offset_odom.p.Norm()
			next_goal_distance = next_goal_offset_odom.p.Norm()
			last_goal_to_next_goal_vector = np.array(list(inter_goal_offset_odom.p))
			last_goal_to_current_pose_vector = np.array(list(last_goal_offset_odom.p))
			last_goal_angle = last_goal_offset_odom.M.GetRPY()[2]
			next_goal_angle = next_goal_offset_odom.M.GetRPY()[2]

			# if it's a distance goal, use distance; if it's a rotation goal, use angle
			if is_turning_goal(last_goal_world, next_goal_world):
				# this isn't used - if it's a turning goal we don't make a correction
				turning_goal = True
				u = last_goal_angle / (last_goal_angle - next_goal_angle) # should have opposite signs
			else:
				turning_goal = False
				# proj(goal1 -> robot, goal1 -> goal2) / |goal1 -> goal2|
				u = np.sum(last_goal_to_next_goal_vector * last_goal_to_current_pose_vector) / np.sum(last_goal_to_next_goal_vector**2)

			# Rotation correction
			offsets, correlations = self.calculate_image_pose_offset(self.goal_index, 1+self.search_range)
			if self.goal_index > self.search_range:
				rotation_offsets = offsets[self.search_range:self.search_range+2]
				rotation_correlations = correlations[self.search_range:self.search_range+2]
			else:
				rotation_offsets = offsets[-self.search_range-3:-self.search_range-1]
				rotation_correlations = correlations[-self.search_range-3:-self.search_range-1]

			offset = (1-u) * rotation_offsets[0] + u * rotation_offsets[1]

			rotation_correction = self.rotation_correction_gain * offset
			if turning_goal or max(rotation_correlations) < self.image_recognition_threshold or u < 0 or u > 1:
				rotation_correction = 0.0

			# Along path correction
			if not turning_goal:
				if self.goal_index > self.search_range and self.goal_index < len(self.poses)-self.search_range:
					corr = np.array(correlations[:2*(1+self.search_range)])
					w = np.arange(-0.5-self.search_range,0.6+self.search_range,1)
				elif self.goal_index <= self.search_range:
					reduced_search_range = self.goal_index - 1
					corr = np.array(correlations[:2*(1+reduced_search_range)])
					w = np.arange(-0.5-reduced_search_range,0.6+reduced_search_range,1)
				else:
					reduced_search_range = len(self.poses) - self.goal_index - 1
					corr = np.array(correlations[-2*(1+reduced_search_range):])
					w = np.arange(-0.5-reduced_search_range,0.6+reduced_search_range,1)
				corr_orig = corr.copy()
				corr -= self.image_recognition_threshold
				corr[corr < 0] = 0.0
				s = corr.sum()
				if s > 0:
					corr /= s
				pos = (corr * w).sum() - (u - 0.5)
				path_error = pos

				# pos > 0: images are telling me I'm ahead of where I think I am
				#   need to reduce the length of the current goal by pos
				#   want d -> d - pos
				#   d *= (d - pos) / d
				# pos < 0: images are telling me I'm behind where I think I am
				#   need to increase the length of the current goal by pos
				#   want d -> d + pos
				#   d *= (d + pos) / d

				path_correction_distance = -self.path_correction_gain * path_error * GOAL_DISTANCE_SPACING
				path_correction = (next_goal_distance + path_correction_distance) / next_goal_distance
				if np.isnan(path_correction):
					print(corr, s, w, pos, next_goal_distance)
				if -path_correction_distance > next_goal_distance:
					self.make_new_goal()
					return
			else:
				path_correction = 1.0
				path_correction_distance = 0.0

			self.sum_theta_correction += rotation_correction
			self.sum_path_correction += path_correction_distance

			goal_offset = get_corrected_goal_offset(current_pose_odom, next_goal_odom, rotation_correction, path_correction)
			new_goal = current_pose_odom * goal_offset
			self.update_goal(new_goal, False, turning_goal)

	def do_discrete_correction(self, pose, old_goal_frame_odom, old_goal_frame_world):
		new_goal_frame_world = tf_conversions.fromMsg(self.poses[self.goal_index])
		turning_goal = is_turning_goal(old_goal_frame_world, new_goal_frame_world)
		inter_goal_offset_world = old_goal_frame_world.Inverse() * new_goal_frame_world
		inter_goal_distance_world = inter_goal_offset_world.p.Norm()

		offsets, correlations = self.calculate_image_pose_offset(self.goal_index, self.search_range)
		if self.goal_index >= self.search_range:
			rotation_offset = offsets[self.search_range]
			rotation_correlation = correlations[self.search_range]
		else:
			rotation_offset = offsets[-self.search_range-1]
			rotation_correlation = correlations[-self.search_range-1]

		offset = rotation_offset

		rotation_correction = self.rotation_correction_gain * offset
		if rotation_correlation < self.image_recognition_threshold:
			rotation_correction = 0.0

		if not turning_goal and self.goal_index >= self.search_range and self.goal_index < len(self.poses)-self.search_range:
			corr = np.array(correlations)
			corr -= self.image_recognition_threshold
			corr[corr < 0] = 0.0
			s = corr.sum()
			if s > 0:
				corr /= s
			w = corr * np.arange(-self.search_range,self.search_range+1,1)
			pos = w.sum()
			path_error = pos

			path_correction_distance = -self.path_correction_gain * path_error * GOAL_DISTANCE_SPACING
			path_correction = (GOAL_DISTANCE_SPACING + path_correction_distance) / GOAL_DISTANCE_SPACING
			if np.isnan(path_correction):
				print(corr, s, w, pos, GOAL_DISTANCE_SPACING)
			if -path_correction_distance > GOAL_DISTANCE_SPACING:
				print('PATH CORRECTION ERROR: correction is greater than distance to goal!')
				print('corr = %s; pos = %f, path_correction = %f, goal_distance = %f' % (str(corr),pos,path_correction_distance,GOAL_DISTANCE_SPACING))
				print('path_correction = %f' % path_correction)
				path_correction_distance = -GOAL_DISTANCE_SPACING
				path_correction = 0.0
		else:
			path_correction = 1.0
			path_correction_distance = 0.0

		self.sum_theta_correction = rotation_correction
		self.sum_path_correction = path_correction_distance
		return rotation_correction, path_correction

	def calculate_image_pose_offset(self, image_to_search_index, half_search_range=0):
		if self.last_image is not None:
			match_request = ImageMatchRequest(image_processing.image_to_msg(self.last_image), UInt32(image_to_search_index), UInt32(half_search_range))
			match_response = self.match_image(match_request)

			# positive image offset: query image is shifted left from reference image
			# (normalise the pixel offset by the width of the image)
			return [px_to_rad(offset) for offset in match_response.offsets.data], match_response.correlations.data
		else:
			raise RuntimeError('Localiser: tried to localise before image data is received!')

if __name__ == "__main__":
	rospy.init_node("teach_repeat_localiser")
	localiser = teach_repeat_localiser()
	if localiser.ready:
		localiser.start()
	rospy.spin()
