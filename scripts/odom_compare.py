#!/usr/bin/python

import numpy as np
import cv2
import os
import pickle
import time
import json
import math
import sys
import matplotlib.pyplot as plt
from rospy_message_converter import message_converter
from geometry_msgs.msg import Pose
import tf_conversions

def read_file(filename):
	with open(filename, 'r') as f:
		data = f.read()
	return data

def get_pose_files(dir):
	pose_files = [dir+f for f in os.listdir(dir) if f.endswith('_pose.txt')]
	pose_files.sort()
	return pose_files

def get_ground_truth_files(dir):
	pose_files = [dir+f for f in os.listdir(dir) if f.endswith('_map_to_base_link.txt')]
	pose_files.sort()
	return pose_files

def read_pose_files(pose_files):
	return [tf_conversions.fromMsg(message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',json.loads(read_file(p)))) for p in pose_files]

def read_transform_stamped_files(pose_files):
	transforms = [message_converter.convert_dictionary_to_ros_message('geometry_msgs/TransformStamped',json.loads(read_file(p))).transform for p in pose_files]
	return [tf_conversions.fromMsg(Pose(tf.translation,tf.rotation)).Inverse() for tf in transforms]

def set_first_pose_to_zero(poses):
	first_pose_inverse = poses[0].Inverse()
	return [first_pose_inverse * pose for pose in poses]

def get_pose_x_y_theta(poses):
	x = np.array([pose.p.x() for pose in poses])
	y = np.array([pose.p.y() for pose in poses])
	theta = np.array([pose.M.GetRPY()[2] for pose in poses])
	return x, y, theta

#### #### ####
odom_dirs = ['/home/dominic/Desktop/2020-08-05_11:14:24/']
odom_dirs = [os.path.expanduser(dir) for dir in odom_dirs]

pose_files = [get_pose_files(dir) for dir in odom_dirs]
ground_truth_files = [get_ground_truth_files(dir) for dir in odom_dirs]
poses = [set_first_pose_to_zero(read_pose_files(p)) for p in pose_files]
ground_truths = [set_first_pose_to_zero(read_transform_stamped_files(p)) for p in ground_truth_files]

pose_data = [get_pose_x_y_theta(pose) for pose in poses] + [get_pose_x_y_theta(pose) for pose in ground_truths]

colours = ['#00ff00', '#0000ff', '#ff0000']

for pose_data_list,colour in zip(pose_data,colours):
	plt.quiver(pose_data_list[0],pose_data_list[1],np.cos(pose_data_list[2]),np.sin(pose_data_list[2]), scale=50, color=colour)

# plt.scatter([3.5], [0], s=100, c='#000000', marker='x')
# plt.scatter([3.5,3.5], [0,-2], s=100, c='#000000', marker='x')
# plt.scatter([7.2,7.2,4.8,1.2], [0,-1.8,-1.8,-1.8], s=100, c='#000000', marker='x')
# plt.scatter([3.6], [0], s=100, c='#000000', marker='x')
# plt.title('Outdoors')
plt.legend(['odom','slam'])
plt.axis('equal')
plt.show()
