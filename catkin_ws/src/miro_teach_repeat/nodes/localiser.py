#!/usr/bin/python

import rospy
import numpy as np
import cv2
import os
import math
import json
import datetime
import enum
import threading
from rospy_message_converter import message_converter
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, UInt32
import tf_conversions
# import tf
import tf2_ros

import image_processing
from miro_teach_repeat.msg import Goal
from miro_teach_repeat.srv import ImageMatch, ImageMatchRequest

FIELD_OF_VIEW_DEG = 2*60.6 + 2*27.0
IMAGE_WIDTH = 115.0 # px
CORR_SUB_SAMPLING = 1

GOAL_DISTANCE_SPACING = 0.2 # m
LOOKAHEAD_DISTANCE_RATIO = 0.65 # x GOAL_DISTANCE_SPACING
TURNING_TARGET_RANGE_DISTANCE_RATIO = 0.5 # x GOAL_DISTANCE_SPACING
# LOOKAHEAD_DISTANCE_RATIO > TURNING_TARGET_RANGE_DISTANCE_RATIO (otherwise stops and turns on spot when arriving at goal)

class GOAL_STATE(enum.Enum):
	normal_goal = 0
	finished = 1
	restart = 2

def read_file(filename):
	with open(filename, 'r') as f:
		data = f.read()
		return data

def load_poses(pose_files):
	return [message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',json.loads(read_file(f))) for f in pose_files]

def wrapToPi(x):
	return ((x + math.pi) % (2*math.pi)) - math.pi

def normalise_quaternion(q):
	norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
	q.x /= norm
	q.y /= norm
	q.z /= norm
	q.w /= norm

def px_to_deg(px):
	return px * FIELD_OF_VIEW_DEG / IMAGE_WIDTH / CORR_SUB_SAMPLING

def deg_to_px(deg):
	return deg * IMAGE_WIDTH * CORR_SUB_SAMPLING / FIELD_OF_VIEW_DEG

def px_to_rad(px):
	return px * math.radians(FIELD_OF_VIEW_DEG) / IMAGE_WIDTH / CORR_SUB_SAMPLING

def rad_to_px(rad):
	return rad * IMAGE_WIDTH * CORR_SUB_SAMPLING / math.radians(FIELD_OF_VIEW_DEG)

def delta_frame_in_bounds(delta_frame):
	distance = delta_frame.p.Norm()
	angle = wrapToPi(delta_frame.M.GetRPY()[2])
	if distance < (LOOKAHEAD_DISTANCE_RATIO * GOAL_DISTANCE_SPACING):
		if abs(angle) < math.radians(5):
			return True
		else:
			pass
			print('angle difference from goal = %.2f [rho = %.3f]' % (math.degrees(angle), distance))
	return False

def get_expected_px_offset(offset_frame):
	# return -0.65*math.degrees(offset_frame.M.GetRPY()[2]) + -15*offset_frame.p.y()
	return -rad_to_px(offset_frame.M.GetRPY()[2])

def get_corrected_goal_offset(goal1, goal2, correction_rad, correction_length):
	''' apply corrections to the offset from goal1 to goal2	'''
	goal_offset = goal1.Inverse() * goal2
	goal_offset = tf_conversions.Frame(tf_conversions.Rotation.RotZ(correction_rad)) * goal_offset
	goal_offset.p *= correction_length
	return goal_offset

def is_turning_goal(goal_frame, next_goal_frame):
	diff = goal_frame.Inverse() * next_goal_frame
	dist = diff.p.Norm()
	return dist < (TURNING_TARGET_RANGE_DISTANCE_RATIO * GOAL_DISTANCE_SPACING)

class teach_repeat_localiser:
	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):
		# Wait for ready
		self.ready = not rospy.get_param('/wait_for_ready', False)
		self.global_localisation_init = rospy.get_param('~global_localisation_init', False)
		self.mutex = threading.Lock()

		# Odom
		self.load_dir = os.path.expanduser(rospy.get_param('/data_load_dir', '~/miro/data'))
		if self.load_dir[-1] != '/':
			self.load_dir += '/'
		self.sum_theta_correction = 0.0
		self.sum_path_correction = 0.0
		self.correction_list = [[],[]]
		self.stop_at_end = rospy.get_param('~stop_at_end', True)
		self.discrete_correction = rospy.get_param('~discrete-correction', False)

		# Load pose data
		pose_files = [self.load_dir+f for f in os.listdir(self.load_dir) if f[-9:] == '_pose.txt']
		pose_files.sort()
		self.poses = load_poses(pose_files)
		self.goal_index = 0
		self.goal = tf_conversions.toMsg(tf_conversions.Frame())
		self.last_goal = tf_conversions.toMsg(tf_conversions.Frame())
		self.goal_plus_lookahead = tf_conversions.toMsg(tf_conversions.Frame())
		self.last_odom_pose = None
		self.zero_odom_offset = None
		global GOAL_DISTANCE_SPACING, LOOKAHEAD_DISTANCE_RATIO, TURNING_TARGET_RANGE_DISTANCE_RATIO
		GOAL_DISTANCE_SPACING = rospy.get_param('/goal_pose_seperation', 0.2)
		LOOKAHEAD_DISTANCE_RATIO = rospy.get_param('/lookahead_distance_ratio', 0.65)
		TURNING_TARGET_RANGE_DISTANCE_RATIO = rospy.get_param('/turning_target_range_distance_ratio', 0.5)

		# Image
		self.resize = image_processing.make_size(height=rospy.get_param('/image_resize_height', None), width=rospy.get_param('/image_resize_width', None))
		if self.resize[1] is not None:
			global IMAGE_WIDTH
			IMAGE_WIDTH = self.resize[1]
		if self.resize[0] is None and self.resize[1] is None:
			self.resize = None
		self.last_image = None
		self.first_img_seq = 0
		global CORR_SUB_SAMPLING, FIELD_OF_VIEW_DEG
		self.image_recognition_threshold = rospy.get_param('/image_recognition_threshold', 0.1)
		CORR_SUB_SAMPLING = rospy.get_param('/image_subsampling', 1)
		FIELD_OF_VIEW_DEG = rospy.get_param('/image_field_of_view_width_deg', 2*60.6 + 2*27.0)
		FIELD_OF_VIEW_RAD = math.radians(FIELD_OF_VIEW_DEG)
		self.search_range = rospy.get_param('~search-range', 1)

		# camera calibration
		self.left_cal_file = rospy.get_param('/calibration_file_left', None)
		self.right_cal_file = rospy.get_param('/calibration_file_right', None)

		# data saving
		self.save_dir = os.path.expanduser(rospy.get_param('/data_save_dir', '~/miro/data/follow-straight_tests/5'))
		if self.save_dir[-1] != '/':
			self.save_dir += '/'
		if not os.path.isdir(self.save_dir):
			os.makedirs(self.save_dir)
		# if not os.path.isdir(self.save_dir+'full/'):
		# 	os.makedirs(self.save_dir+'full/')
		if not os.path.isdir(self.save_dir+'norm/'):
			os.makedirs(self.save_dir+'norm/')
		if not os.path.isdir(self.save_dir+'pose/'):
			os.makedirs(self.save_dir+'pose/')
		if not os.path.isdir(self.save_dir+'offset/'):
			os.makedirs(self.save_dir+'offset/')
		if not os.path.isdir(self.save_dir+'correction/'):
			os.makedirs(self.save_dir+'correction/')
		self.goal_number = 0
		
	def setup_publishers(self):	
		self.goal_pub = rospy.Publisher('goal', Goal, queue_size=1)

		rospy.wait_for_service('match_image')
		self.match_image = rospy.ServiceProxy('match_image', ImageMatch, persistent=True)

	def setup_subscribers(self):
		if not self.ready:
			self.sub_ready = rospy.Subscriber("ready", Bool, self.on_ready, queue_size=1)
		self.sub_odom = rospy.Subscriber("odom", Odometry, self.process_odom_data, queue_size=1)
		self.sub_images = rospy.Subscriber('image', Image, self.process_image_data, queue_size=1, buff_size=2**22)
		self.tfBuffer = tf2_ros.Buffer()
		self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

	def on_ready(self, msg):
		if msg.data:
			if not self.ready:
				self.start()
			self.ready = True

	def start(self):
		if self.global_localisation_init:
			if self.last_odom_pose is None or self.last_image is None:
				print('Global localisation - waiting for first odom and image messages')
				while self.last_odom_pose is None or self.last_image is None:
					pass
			
			offsets, correlations = self.calculate_image_pose_offset(0, len(self.poses))
			best_match = np.argmax(correlations)
			if best_match == 0 or (best_match == 1 and correlations[0] > correlations[2]):
				if self.poses[0].position.x == 0 and self.poses[0].position.y == 0 and tf_conversions.fromMsg(self.poses[0]).M.GetRPY()[2] == 0:
					# the first pose is at 0,0,0 so we ignore it and set the first goal to the next pose
					print('Localiser: starting at goal 1, goal 0 = [0,0,0]')
					self.goal_index = 1
				else:
					self.goal_index = 0
				goal_pose = tf_conversions.Frame()
				odom_pose = tf_conversions.fromMsg(self.last_odom_pose)
				self.zero_odom_offset = tf_conversions.fromMsg(self.last_odom_pose)
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
		else:
			if self.poses[0].position.x == 0 and self.poses[0].position.y == 0 and tf_conversions.fromMsg(self.poses[0]).M.GetRPY()[2] == 0:
				# the first pose is at 0,0,0 so we ignore it and set the first goal to the next pose
				print('Localiser: starting at goal 1, goal 0 = [0,0,0]')
				self.goal_index = 1
			else:
				self.goal_index = 0
		self.update_goal(tf_conversions.fromMsg(self.poses[self.goal_index]))

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
		try:
			trans = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
			trans_as_text = json.dumps(message_converter.convert_ros_message_to_dictionary(trans))
			with open(self.save_dir+('pose/%06d_map_to_base_link.txt' % self.goal_number), 'w') as tf_trans_file:
				tf_trans_file.write(trans_as_text)
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print('Could not lookup transform from /map to /base_link')
			pass
		
		# save current pose info
		message_as_text = json.dumps(message_converter.convert_ros_message_to_dictionary(pose))
		with open(self.save_dir+('pose/%06d_pose.txt' % self.goal_number), 'w') as pose_file:
			pose_file.write(message_as_text)
		# save current offset
		offset = tf_conversions.toMsg(goal_odom.Inverse() * goal_world)
		message_as_text = json.dumps(message_converter.convert_ros_message_to_dictionary(offset))
		with open(self.save_dir+('offset/%06d_offset.txt' % self.goal_number), 'w') as offset_file:
			offset_file.write(message_as_text)
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
		if self.ready and self.last_image is not None:
			self.mutex.acquire()
			if self.last_odom_pose is None:
				self.zero_odom_offset = tf_conversions.fromMsg(msg.pose.pose)
			msg = self.subtract_odom(msg, self.zero_odom_offset)
			self.last_odom_pose = msg.pose.pose

			current_pose_odom = tf_conversions.fromMsg(msg.pose.pose)
			current_goal_frame_odom = tf_conversions.fromMsg(self.goal)
			current_goal_plus_lookahead_frame_odom = tf_conversions.fromMsg(self.goal_plus_lookahead)
			old_goal_frame_world = tf_conversions.fromMsg(self.poses[self.goal_index])

			delta_frame = current_pose_odom.Inverse() * current_goal_plus_lookahead_frame_odom

			if delta_frame_in_bounds(delta_frame):
				old_goal_index = self.goal_index

				state = self.update_goal_index()
				
				if state == GOAL_STATE.finished:
					return
				
				if state == GOAL_STATE.restart:
					# don't have a big offset from the end of the path, back to the start
					old_goal_frame_world = tf_conversions.Frame()

				if self.discrete_correction:
					self.do_discrete_correction(msg.pose.pose, current_goal_frame_odom, old_goal_frame_world)
				else:
					new_goal_frame_world = tf_conversions.fromMsg(self.poses[self.goal_index])
					turning_goal = is_turning_goal(old_goal_frame_world, new_goal_frame_world)
					goal_offset = get_corrected_goal_offset(old_goal_frame_world, new_goal_frame_world, 0.0, 1.0)
					new_goal = current_goal_frame_odom * goal_offset

					sum_path_correction_ratio = (GOAL_DISTANCE_SPACING + self.sum_path_correction) / GOAL_DISTANCE_SPACING
					self.update_goal(new_goal, True, turning_goal)
					self.save_data_at_goal(msg.pose.pose, new_goal, new_goal_frame_world, self.sum_theta_correction, sum_path_correction_ratio)
					self.goal_number += 1
					print('[%d] theta [%f]\tpath [%f]' % (old_goal_index, math.degrees(self.sum_theta_correction), sum_path_correction_ratio))
					if turning_goal:
						print('turning goal:')
					self.sum_theta_correction = 0
					self.sum_path_correction = 0.0
					self.correction_list = [[],[]]
			self.mutex.release()
		elif self.global_localisation_init:
			self.last_odom_pose = msg.pose.pose

	def process_image_data(self, msg):
		if self.ready:
			n = msg.header.seq

			full_image = image_processing.msg_to_image(msg)
			normalised_image = image_processing.patch_normalise_image(full_image, (9,9), resize=self.resize)
			
			# cv2.imwrite(self.save_dir+('full/%06d.png' % n), full_image)
			cv2.imwrite(self.save_dir+('norm/%06d.png' % n), np.uint8(255.0 * (1 + normalised_image) / 2.0))
			
			self.mutex.acquire()
			self.last_image = normalised_image

			if not self.discrete_correction:
				self.do_continuous_correction()
			self.mutex.release()
		elif self.global_localisation_init:
			self.last_image = image_processing.patch_normalise_image(image_processing.msg_to_image(msg), (9,9), resize=self.resize)

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
		pose_frame_looahead = pose_frame * lookahead

		goal.pose.pose.position.x = pose_frame_looahead.p.x()
		goal.pose.pose.position.y = pose_frame_looahead.p.y()
		goal.pose.pose.orientation = tf_conversions.toMsg(pose_frame_looahead).orientation

		goal.stop_at_goal.data = stop_at_goal
		self.goal_pub.publish(goal)
		self.goal_plus_lookahead = tf_conversions.toMsg(original_pose_frame_lookahead)

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
				# u = last_goal_distance / (last_goal_distance + next_goal_distance)
				u = np.sum(last_goal_to_next_goal_vector * last_goal_to_current_pose_vector) / np.sum(last_goal_to_next_goal_vector**2)

			offsets, correlations = self.calculate_image_pose_offset(self.goal_index, 1+self.search_range)
			if self.goal_index > self.search_range:
				rotation_offsets = offsets[self.search_range:self.search_range+2]
				rotation_correlations = correlations[self.search_range:self.search_range+2]
			else:
				rotation_offsets = offsets[-self.search_range-3:-self.search_range-1]
				rotation_correlations = correlations[-self.search_range-3:-self.search_range-1]

			offset = (1-u) * rotation_offsets[0] + u * rotation_offsets[1]

			K = 0.01
			correction_rad = K * offset
			if turning_goal or max(rotation_correlations) < self.image_recognition_threshold or u < 0 or u > 1:
				correction_rad = 0.0

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

				K2 = 0.01
				path_correction_distance = -K2 * path_error * GOAL_DISTANCE_SPACING
				path_correction = (next_goal_distance + path_correction_distance) / next_goal_distance
				if np.isnan(path_correction):
					print(corr, s, w, pos, next_goal_distance)
				if -path_correction_distance > next_goal_distance:
					print('PATH CORRECTION ERROR: correction is greater than distance to goal!')
					print('corr = %s; pos = %f, path_correction = %f, goal_distance = %f' % (str(corr),pos,path_correction_distance,next_goal_distance))
					print('path_correction = %f' % path_correction)
					path_correction_distance = -next_goal_distance
					path_correction = 0.0
			else:
				path_correction = 1.0
				path_correction_distance = 0.0

			goal_offset = get_corrected_goal_offset(current_pose_odom, next_goal_odom, correction_rad, path_correction)
			new_goal = current_pose_odom * goal_offset

			self.update_goal(new_goal, False, turning_goal)
			self.sum_theta_correction += correction_rad
			self.sum_path_correction += path_correction_distance
			self.correction_list[0].append(correction_rad)
			self.correction_list[1].append(path_correction_distance)
			# new_lookahead_goal = tf_conversions.fromMsg(self.goal_plus_lookahead)
			# d = current_pose_odom.Inverse() * new_lookahead_goal
			# print('pos = %f, d = %f' % (pos, d.p.Norm()))

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

		K = 0.1
		correction_rad = K * offset
		if rotation_correlation < self.image_recognition_threshold:
			correction_rad = 0.0

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

			K2 = 0.5
			if inter_goal_distance_world + K2 * path_error * GOAL_DISTANCE_SPACING < 0:
				path_correction = 0.0
			else:
				path_correction = (inter_goal_distance_world - K2 * path_error * GOAL_DISTANCE_SPACING) / inter_goal_distance_world
			if np.isnan(path_correction):
				print(corr, s, w, pos, inter_goal_distance_world)
			# Note: need a check for if abs(path_error) > inter_goal_distance_odom
			# RuntimeWarning: invalid value encountered in double_scalars
		else:
			path_correction = 1.0
			pos = 0.0

		goal_offset = get_corrected_goal_offset(old_goal_frame_world, new_goal_frame_world, correction_rad, path_correction)
		new_goal = old_goal_frame_odom * goal_offset

		self.update_goal(new_goal)
		self.save_data_at_goal(pose, new_goal, new_goal_frame_world, correction_rad, path_correction)
		self.goal_number += 1
		print('last goal theta correction: %f' % math.degrees(correction_rad))
		print('last goal path correction: %f' % (path_correction))

	def calculate_image_pose_offset(self, image_to_search_index, half_search_range=None):
		HALF_SEARCH_RANGE = 1
		if half_search_range is None:
			half_search_range = HALF_SEARCH_RANGE

		if self.last_image is not None:
			match_request = ImageMatchRequest(image_processing.image_to_msg(self.last_image), UInt32(image_to_search_index), UInt32(half_search_range))
			match_response = self.match_image(match_request)

			# positive image offset: query image is shifted left from reference image
			# this means we have done a right (negative turn) which we should correct with a positive turn
			# positive offset -> positive turn, gain = positive
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