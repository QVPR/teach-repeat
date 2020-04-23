#!/usr/bin/python

import rospy
import numpy as np
import os
import time
import json
import math
from rospy_message_converter import message_converter
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt32
import tf_conversions


import image_processing
from miro_teach_repeat.msg import ImageAndPose
from miro_teach_repeat.srv import PoseOffset

class miro_odom_follower:

	def __init__(self):		
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):
		self.load_dir = os.path.expanduser(rospy.get_param('miro_data_load_dir', '~/miro/data'))
		if self.load_dir[-1] != '/':
			self.load_dir += '/'

		pose_files = [self.load_dir+f for f in os.listdir(self.load_dir) if f[-9:] == '_pose.txt']
		pose_files.sort()

		self.poses = self.load_poses(pose_files)
		self.goal_index = 0
		self.goal = self.poses[self.goal_index]

		self.localise = rospy.get_param('~localise', False)

	def setup_publishers(self):
		self.goal_pub = rospy.Publisher('/miro/control/goal', PoseStamped, queue_size=0)

		if self.localise:
			rospy.wait_for_service('/miro/get_image_pose_offset')
			self.get_image_pose_offset = rospy.ServiceProxy('/miro/get_image_pose_offset', PoseOffset, persistent=True)

	def setup_subscribers(self):
		self.sub_odom = rospy.Subscriber("/miro/sensors/odom/integrated", Odometry, self.process_odom_data, queue_size=1)

	def load_poses(self, pose_files):
		return [message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',json.loads(self.read_file(f))) for f in pose_files]

	def read_file(self, filename):
		with open(filename, 'r') as f:
			data = f.read()
		return data

	def process_odom_data(self, msg):
		current_frame_odom = tf_conversions.fromMsg(msg.pose.pose)
		current_goal_frame_odom = tf_conversions.fromMsg(self.goal)
		old_goal_frame_world = tf_conversions.fromMsg(self.poses[self.goal_index])

		delta_frame = current_frame_odom.Inverse() * current_goal_frame_odom

		if delta_frame.p.Norm() < 0.10:
			old_goal_index = self.goal_index
			self.goal_index += 1
			if self.goal_index == len(self.poses):
				self.goal_index = len(self.poses)-1
				# self.goal_index = 0

			if self.localise:
				image_theta_offset = self.get_image_pose_offset(UInt32(data=old_goal_index)).poseThetaOffset.data
				new_goal_frame_world = tf_conversions.fromMsg(self.poses[self.goal_index])

				# old target -> new target in frame of old target (x = forwards)
				goal_offset = old_goal_frame_world.Inverse() * new_goal_frame_world
				# rotate to odom frame (x = positive odom) [same frame as current_goal]
				goal_offset.p = tf_conversions.Rotation.RotZ(current_goal_frame_odom.M.GetRPY()[2]) * goal_offset.p
				# rotate the goal offset by the rotational correction
				goal_offset_corrected = tf_conversions.Frame(tf_conversions.Rotation.RotZ(image_theta_offset)) * goal_offset
				# add the corrected offset to the current goal
				new_goal_odom = tf_conversions.Frame(goal_offset_corrected.M * current_goal_frame_odom.M, goal_offset_corrected.p + current_goal_frame_odom.p)

				self.goal = tf_conversions.toMsg(new_goal_odom)
				self.publish_goal(self.goal, 0.1)
			else:
				self.goal = self.poses[self.goal_index]
				self.publish_goal(self.goal, 0.1)
	
	def publish_goal(self, pose, lookahead_distance=0.0):
		goal = PoseStamped()
		goal.header.stamp = rospy.Time.now()
		goal.header.frame_id = "odom"
		theta = tf_conversions.fromMsg(pose).M.GetRPY()[2]
		goal.pose.position.x = pose.position.x + lookahead_distance * math.cos(theta)
		goal.pose.position.y = pose.position.y + lookahead_distance * math.sin(theta)
		goal.pose.orientation = pose.orientation
		self.goal_pub.publish(goal)

if __name__ == "__main__":
	rospy.init_node("miro_odom_follower")
	follower = miro_odom_follower()
	follower.publish_goal(follower.poses[0])
	rospy.spin()
