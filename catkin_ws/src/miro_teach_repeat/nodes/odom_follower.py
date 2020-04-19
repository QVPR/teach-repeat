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
import tf_conversions


import image_processing
from miro_teach_repeat.msg import ImageAndPose
from miro_teach_repeat.srv import ImageMatch

class miro_odom_follower:

	def __init__(self):		
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):
		self.save_dir = os.path.expanduser(rospy.get_param('~save_dir', '~/miro/data'))
		if self.save_dir[-1] != '/':
			self.save_dir += '/'

		pose_files = [self.save_dir+f for f in os.listdir(self.save_dir) if f[-9:] == '_pose.txt']
		pose_files.sort()

		self.poses = self.load_poses(pose_files)
		self.current_goal = 0

	def setup_publishers(self):
		self.goal_pub = rospy.Publisher('/miro/control/goal', PoseStamped, queue_size=0)

	def setup_subscribers(self):
		self.sub_odom = rospy.Subscriber("/miro/sensors/odom/integrated", Odometry, self.process_odom_data, queue_size=1)

	def load_poses(self, pose_files):
		return [message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',json.loads(self.read_file(f))) for f in pose_files]

	def read_file(self, filename):
		with open(filename, 'r') as f:
			data = f.read()
		return data

	def process_odom_data(self, msg):
		current_frame = tf_conversions.fromMsg(msg.pose.pose)
		goal_frame = tf_conversions.fromMsg(self.poses[self.current_goal])

		delta_frame = current_frame.Inverse() * goal_frame

		if delta_frame.p.Norm() < 0.10:
			if self.current_goal+1 < len(self.poses):
				self.current_goal += 1
				if self.current_goal+1 == len(self.poses):
					self.publish_goal(self.poses[self.current_goal], 0.0)
				else:
					self.publish_goal(self.poses[self.current_goal])
			else:
				self.current_goal = 0
				self.publish_goal(self.poses[self.current_goal])
	
	def publish_goal(self, pose, lookahead_distance=0.2):
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
	time.sleep(1) # seemed to solve issue with goal being published before listener ready
	follower.publish_goal(follower.poses[0])
	rospy.spin()
