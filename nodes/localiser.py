#!/usr/bin/python

import rospy
import numpy as np
import cv2
import os
import math
import json
import threading
from rospy_message_converter import message_converter
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, UInt32
import tf_conversions
import tf

import image_processing
from miro_onboard.msg import CompressedImageSynchronised
from miro_teach_repeat.msg import Goal
from miro_teach_repeat.srv import ImageMatch, ImageMatchRequest

IMAGE_RECOGNITION_THRESHOLD = 0.1

FIELD_OF_VIEW_DEG = 2*60.6 + 2*27.0 # deg
FIELD_OF_VIEW_RAD = math.radians(FIELD_OF_VIEW_DEG)
IMAGE_WIDTH = 115.0 # px
GOAL_DISTANCE_SPACING = 0.2 # m

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
	return px * FIELD_OF_VIEW_DEG / IMAGE_WIDTH

def deg_to_px(deg):
	return deg * IMAGE_WIDTH / FIELD_OF_VIEW_DEG

def px_to_rad(px):
	return px * FIELD_OF_VIEW_RAD / IMAGE_WIDTH

def rad_to_px(rad):
	return rad * IMAGE_WIDTH / FIELD_OF_VIEW_RAD

def delta_frame_in_bounds(delta_frame):
	distance = delta_frame.p.Norm()
	angle = wrapToPi(delta_frame.M.GetRPY()[2])
	if distance < 0.10:
		if abs(angle) < math.radians(5):
			return True
		else:
			print('angle difference from goal = %.2f [rho = %.3f]' % (math.degrees(angle), distance))
	return False

def get_expected_px_offset(offset_frame):
	# return -0.65*math.degrees(offset_frame.M.GetRPY()[2]) + -15*offset_frame.p.y()
	return -rad_to_px(offset_frame.M.GetRPY()[2])

def get_corrected_goal_offset(goal1, goal2, correction_rad, path_correction):
	# # old target -> new target in frame of old target (x = forwards)
	# goal_offset = old_goal_frame_world.Inverse() * new_goal_frame_world
	# # rotate to odom frame (x = positive odom) [same frame as current_goal]
	# goal_offset.p = tf_conversions.Rotation.RotZ(current_goal_frame_odom.M.GetRPY()[2]) * goal_offset.p
	# # rotate the goal offset by the rotational correction
	# goal_offset_corrected = tf_conversions.Frame(tf_conversions.Rotation.RotZ(correction_rad)) * goal_offset
	# # extend or retract the goal using image-based along-path offset
	# goal_offset_corrected.p *= image_path_offset
	# # add the corrected offset to the current goal
	# new_goal_odom = tf_conversions.Frame(goal_offset_corrected.M * current_goal_frame_odom.M, goal_offset_corrected.p + current_goal_frame_odom.p)

	goal_offset = goal1.Inverse() * goal2
	goal_offset = tf_conversions.Frame(tf_conversions.Rotation.RotZ(correction_rad)) * goal_offset
	goal_offset.p *= path_correction
	return goal_offset

def is_turning_goal(goal_frame, next_goal_frame):
	diff = goal_frame.Inverse() * next_goal_frame
	dist = diff.p.Norm()
	return dist < (GOAL_DISTANCE_SPACING / 2)

class miro_localiser:
	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):
		self.ready = not rospy.get_param('/wait_for_ready', False)
		self.mutex = threading.Lock()

		# Odom
		self.load_dir = os.path.expanduser(rospy.get_param('miro_data_load_dir', '~/miro/data'))
		if self.load_dir[-1] != '/':
			self.load_dir += '/'

		pose_files = [self.load_dir+f for f in os.listdir(self.load_dir) if f[-9:] == '_pose.txt']
		pose_files.sort()

		self.poses = load_poses(pose_files)
		self.goal_index = 0
		self.goal = tf_conversions.toMsg(tf_conversions.Frame())
		self.last_goal = tf_conversions.toMsg(tf_conversions.Frame())
		self.goal_plus_lookahead = tf_conversions.toMsg(tf_conversions.Frame())

		self.stop_at_end = rospy.get_param('~stop_at_end', True)

		self.last_odom = None
		self.sum_theta_correction = 0.0
		self.sum_path_correction = 1.0

		# Image
		self.resize = image_processing.make_size(height=rospy.get_param('/image_resize_height', None), width=rospy.get_param('/image_resize_width', None))
		if self.resize[0] is not None:
			IMAGE_WIDTH = self.resize[0]
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
		if not os.path.isdir(self.save_dir+'pose/'):
			os.makedirs(self.save_dir+'pose/')
		if not os.path.isdir(self.save_dir+'offset/'):
			os.makedirs(self.save_dir+'offset/')
		if not os.path.isdir(self.save_dir+'correction/'):
			os.makedirs(self.save_dir+'correction/')

		self.goal_number = 0
		
	def setup_publishers(self):	
		self.goal_pub = rospy.Publisher('/miro/control/goal', Goal, queue_size=1)

		rospy.wait_for_service('/miro/match_image')
		self.match_image = rospy.ServiceProxy('/miro/match_image', ImageMatch, persistent=True)
		self.tf_pub = tf.TransformBroadcaster()

	def setup_subscribers(self):
		if not self.ready:
			self.sub_ready = rospy.Subscriber("/miro/ready", Bool, self.on_ready, queue_size=1)
		self.sub_odom = rospy.Subscriber("/miro/sensors/odom/integrated", Odometry, self.process_odom_data, queue_size=1)
		self.sub_images = rospy.Subscriber('/miro/sensors/cam/both/compressed', CompressedImageSynchronised, self.process_image_data, queue_size=1, buff_size=2**22)

	def on_ready(self, msg):
		if msg.data:
			if not self.ready:
				self.update_goal(tf_conversions.fromMsg(self.poses[0]))
			self.ready = True

	def update_goal_index(self):
		self.goal_index += 1
		if self.goal_index == len(self.poses):
			if self.stop_at_end:
				self.goal_index = len(self.poses) - 1
				# save image when arriving at the final goal
				if self.goal_number < len(self.poses):
					self.calculate_image_pose_offset(self.goal_index)
					self.goal_number += 1
				return True
			else:
				self.goal_index = 0 # repeat the path (loop)
		return False

	def save_data_at_goal(self, pose, goal_odom, goal_world, theta_offset, path_offset):
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
		self.tf_pub.sendTransform((offset.position.x,offset.position.y,offset.position.z),(offset.orientation.x,offset.orientation.y,offset.orientation.z,offset.orientation.w),rospy.Time.now(),'map','odom')
		# publish current corrections
		message_as_text = json.dumps({'theta_offset':theta_offset, 'path_offset':path_offset})
		with open(self.save_dir+('correction/%06d_correction.txt' % self.goal_number), 'w') as correction_file:
			correction_file.write(message_as_text)

	def process_odom_data(self, msg):
		if self.ready and self.last_image is not None:
			self.mutex.acquire()
			self.last_odom = msg.pose.pose

			current_frame_odom = tf_conversions.fromMsg(msg.pose.pose)
			current_goal_frame_odom = tf_conversions.fromMsg(self.goal)
			current_goal_plus_lookahead_frame_odom = tf_conversions.fromMsg(self.goal_plus_lookahead)
			old_goal_frame_world = tf_conversions.fromMsg(self.poses[self.goal_index])

			delta_frame = current_frame_odom.Inverse() * current_goal_plus_lookahead_frame_odom

			if delta_frame_in_bounds(delta_frame):
				old_goal_index = self.goal_index

				if self.update_goal_index(): # at final goal
					return

				new_goal_frame_world = tf_conversions.fromMsg(self.poses[self.goal_index])
				# image_path_offset, image_rad_offset, correlation, best_correlation = self.calculate_image_pose_offset(old_goal_index)

				# known_goal_offset = current_goal_frame_odom.Inverse() * current_frame_odom
				# expected_theta_offset = px_to_rad(get_expected_px_offset(known_goal_offset))
				# correction_rad = image_rad_offset - expected_theta_offset
				# path_correction = image_path_offset

				# if correlation < IMAGE_RECOGNITION_THRESHOLD:
				# 	correction_rad = 0.0
				# if best_correlation < IMAGE_RECOGNITION_THRESHOLD:
				# 	path_correction = 1.0

				# goal_offset = get_corrected_goal_offset(old_goal_frame_world, new_goal_frame_world, correction_rad, path_correction)
				# new_goal = current_goal_frame_odom * goal_offset

				# self.update_goal(new_goal)
				# self.save_data_at_goal(msg.pose.pose, new_goal, new_goal_frame_world, correction_rad, image_path_offset)
				# self.goal_number += 1

				goal_offset = get_corrected_goal_offset(old_goal_frame_world, new_goal_frame_world, 0.0, 1.0)
				new_goal = current_goal_frame_odom * goal_offset

				self.update_goal(new_goal, True, is_turning_goal(old_goal_frame_world, new_goal_frame_world))
				self.save_data_at_goal(msg.pose.pose, new_goal, new_goal_frame_world, self.sum_theta_correction, self.sum_path_correction)
				self.goal_number += 1

				print('last goal theta correction: %f' % math.degrees(self.sum_theta_correction))
				print('last goal path correction: %f' % (self.sum_path_correction))
				self.sum_theta_correction = 0
				self.sum_path_correction = 1.0
			self.mutex.release()

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

			self.mutex.acquire()
			self.last_image = normalised_image

			self.do_continuous_correction()
			self.mutex.release()

	def update_goal(self, goal_frame, new_goal=True, turning_goal=False):
		if new_goal:
			self.last_goal = self.goal

		self.goal = tf_conversions.toMsg(goal_frame)
		normalise_quaternion(self.goal.orientation)

		# if goal is a turning goal, or the last goal - don't set virtual waypoint ahead
		if turning_goal or (self.goal_index == len(self.poses)-1 and self.stop_at_end):
			self.publish_goal(self.goal, 0.0, True)
		else:
			self.publish_goal(self.goal, 0.1, False)

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
		self.goal_plus_lookahead = goal.pose.pose

	def do_continuous_correction(self):
		if self.last_odom is not None and self.goal_index > 0:
			next_goal_world = tf_conversions.fromMsg(self.poses[self.goal_index])
			last_goal_world = tf_conversions.fromMsg(self.poses[self.goal_index-1])
			last_goal_odom = tf_conversions.fromMsg(self.last_goal)
			next_goal_odom = tf_conversions.fromMsg(self.goal)
			current_frame_odom = tf_conversions.fromMsg(self.last_odom)

			next_goal_offset_odom = next_goal_odom.Inverse() * current_frame_odom
			last_goal_offset_odom = last_goal_odom.Inverse() * current_frame_odom
			inter_goal_offset_odom = last_goal_odom.Inverse() * next_goal_odom
			inter_goal_distance_odom = inter_goal_offset_odom.p.Norm()

			next_goal_distance = next_goal_offset_odom.p.Norm()
			last_goal_distance = last_goal_offset_odom.p.Norm()
			next_goal_angle = next_goal_offset_odom.M.GetRPY()[2]
			last_goal_angle = last_goal_offset_odom.M.GetRPY()[2]

			# if it's a distance goal, use distance; if it's a rotation goal, use angle
			if is_turning_goal(last_goal_world, next_goal_world):
				turning_goal = True
				u = last_goal_angle / (last_goal_angle + next_goal_angle)
			else:
				turning_goal = False
				u = last_goal_distance / (last_goal_distance + next_goal_distance)

			search_range = 1
			offsets, correlations = self.calculate_image_pose_offset(self.goal_index, 1+search_range, return_all=True)
			if self.goal_index > search_range:
				rotation_offsets = offsets[search_range:search_range+2]
				rotation_correlations = correlations[search_range:search_range+2]
			else:
				rotation_offsets = offsets[-search_range-3:-search_range-1]
				rotation_correlations = correlations[-search_range-3:-search_range-1]

			offset = (1-u) * rotation_offsets[0] + u * rotation_offsets[1]

			K = 0.01
			correction_rad = K * offset
			if max(rotation_correlations) < IMAGE_RECOGNITION_THRESHOLD:
				correction_rad = 0.0

			if not turning_goal and self.goal_index > search_range and self.goal_index < len(self.poses)-search_range:
				corr = np.array(correlations[:2*(1+search_range)])
				corr -= IMAGE_RECOGNITION_THRESHOLD
				corr[corr < 0] = 0.0
				s = corr.sum()
				if s > 0:
					corr /= s
				w = corr * np.arange(-0.5-search_range,0.6+search_range,1)
				pos = w.sum()
				path_error = pos

				K2 = 0.01
				path_correction = inter_goal_distance_odom / (inter_goal_distance_odom + K2 * path_error)
				if np.isnan(path_correction):
					print(corr, s, w, pos, inter_goal_distance_odom)
				# Note: need a check for if abs(path_error) > inter_goal_distance_odom
				# RuntimeWarning: invalid value encountered in double_scalars
			else:
				path_correction = 1.0
				pos = 0.0

			goal_offset = get_corrected_goal_offset(last_goal_odom, next_goal_odom, correction_rad, path_correction)
			new_goal = last_goal_odom * goal_offset

			self.update_goal(new_goal, False, turning_goal)
			self.sum_theta_correction += correction_rad
			self.sum_path_correction *= path_correction
			# new_lookahead_goal = tf_conversions.fromMsg(self.goal_plus_lookahead)
			# d = current_frame_odom.Inverse() * new_lookahead_goal
			# print('pos = %f, d = %f' % (pos, d.p.Norm()))

	def calculate_image_pose_offset(self, image_to_search_index, half_search_range=None, return_all=False):
		HALF_SEARCH_RANGE = 1
		if half_search_range is None:
			half_search_range = HALF_SEARCH_RANGE

		if self.last_image is not None:
			match_request = ImageMatchRequest(image_processing.image_to_msg(self.last_image), UInt32(image_to_search_index), UInt32(half_search_range))
			match_response = self.match_image(match_request)

			if image_to_search_index >= half_search_range:
				centre_image_index = half_search_range
			else:
				centre_image_index = image_to_search_index

			image_match_offset = match_response.offsets.data[centre_image_index]
			image_match_corr = match_response.correlations.data[centre_image_index]
			best_match = np.argmax(match_response.correlations.data)

			path_offset_magnitude = best_match - centre_image_index
			if path_offset_magnitude > 0:
				path_offset = 0.5 ** path_offset_magnitude
			elif path_offset_magnitude < 0:
				path_offset = 1.5 ** (-path_offset_magnitude)
			else:
				path_offset = 1.0		

			# positive image offset: query image is shifted left from reference image
			# this means we have done a right (negative turn) which we should correct with a positive turn
			# positive offset -> positive turn, gain = positive
			# (normalise the pixel offset by the width of the image)
			theta_offset = px_to_rad(image_match_offset)

			if return_all:
				return [px_to_rad(offset) for offset in match_response.offsets.data], match_response.correlations.data
			else:
				return path_offset, theta_offset, image_match_corr, match_response.correlations.data[best_match]
		else:
			raise RuntimeError('Localiser: tried to localise before image data is received!')

if __name__ == "__main__":
	rospy.init_node("miro_localiser")
	localiser = miro_localiser()
	if localiser.ready:
		localiser.update_goal(tf_conversions.fromMsg(localiser.poses[0]))
	rospy.spin()