#!/usr/bin/env python

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
	pose_files = [dir+f for f in os.listdir(dir) if f.endswith('_pose.txt') or f.endswith('_goal.txt')]
	pose_files.sort()
	return pose_files

def get_ground_truth_files(dir):
	pose_files = np.array([f for f in os.listdir(dir) if f.endswith('_map_to_base_link.txt')])
	nums = [int(s.split('_')[0]) for s in pose_files]
	idx = np.argsort(nums)
	pose_files = [dir+f for f in pose_files[idx]]
	return pose_files

def read_pose_files(pose_files):
	return [tf_conversions.fromMsg(message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',json.loads(read_file(p)))) for p in pose_files]

def read_transform_stamped_files(pose_files):
	transforms = [message_converter.convert_dictionary_to_ros_message('geometry_msgs/TransformStamped',json.loads(read_file(p))).transform for p in pose_files]
	return [tf_conversions.fromMsg(Pose(tf.translation,tf.rotation)) for tf in transforms]

def set_first_pose_to_zero(poses):
	first_pose_inverse = poses[0].Inverse()
	return [first_pose_inverse * pose for pose in poses]

def get_pose_x_y_theta(poses):
	x = np.array([pose.p.x() for pose in poses])
	y = np.array([pose.p.y() for pose in poses])
	theta = np.array([pose.M.GetRPY()[2] for pose in poses])
	return x, y, theta

def pad_pose_x_y_to_length(poses, length):
	x = np.hstack((np.full((length - len(poses[0])), np.nan), poses[0], np.full((1), np.nan)))
	y = np.hstack((np.full((length - len(poses[1])), np.nan), poses[1], np.full((1), np.nan)))
	theta = np.hstack((np.full((length - len(poses[2])), np.nan), poses[2], np.full((1), np.nan)))
	return x, y, theta

def wrapToPi(x):
	'''wrap angle to between +pi and -pi'''
	return ((x + math.pi) % (2*math.pi)) - math.pi

#### #### ####
odom_dirs = ['/home/dominic/Desktop/2020-08-18_13:12:42/','/home/dominic/Desktop/2020-08-18_13:12:42-repeat1/','/home/dominic/Desktop/2020-08-18_13:12:42-repeat2/']
odom_dirs = [os.path.expanduser(dir) for dir in odom_dirs]

# pose_files = [get_pose_files(dir) for dir in odom_dirs]
ground_truth_files = [get_ground_truth_files(dir) for dir in odom_dirs]
# poses = [set_first_pose_to_zero(read_pose_files(p)) for p in pose_files if p != []]
ground_truths = [read_transform_stamped_files(p) for p in ground_truth_files if p != []]

# pose_data = [get_pose_x_y_theta(pose) for pose in poses] + [get_pose_x_y_theta(pose) for pose in ground_truths]
pose_data = [get_pose_x_y_theta(pose) for pose in ground_truths]

for i in range(1, len(pose_data)):
	pose_data[i] = pad_pose_x_y_to_length(pose_data[i], len(pose_data[0][0])-1)

colours = ['#44dd44', '#4444dd', '#dd4444']

for pose_data_list,colour in zip(pose_data,colours):
	plt.quiver(pose_data_list[0],pose_data_list[1],np.cos(pose_data_list[2]),np.sin(pose_data_list[2]), scale=50, color=colour)
plt.legend(['teach', 'repeat filtered odom', 'repeat raw odom'])

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)

# plt.figure()
for pose_data_list,colour in zip(pose_data[1:],colours[1:]):
	ax1.plot(pose_data_list[0]-pose_data[0][0], color=colour)
	ax2.plot(pose_data_list[1]-pose_data[0][1], color=colour)
	ax3.plot(180/math.pi*(wrapToPi(pose_data_list[2]-pose_data[0][2])), color=colour)

# plt.figure()
# for pose_data_list,colour in zip(pose_data,colours):
# 	plt.plot(pose_data_list[1], color=colour)

# plt.figure()
# for pose_data_list,colour in zip(pose_data,colours):
# 	plt.plot(pose_data_list[2], color=colour)

# plt.figure()
# plt.plot(pose_data[0][0])
# plt.figure()
# plt.plot(pose_data[0][1])
# plt.figure()
# plt.plot(pose_data[0][2])

# plt.scatter([3.5], [0], s=100, c='#000000', marker='x')
# plt.scatter([3.5,3.5], [0,-2], s=100, c='#000000', marker='x')
# plt.scatter([7.2,7.2,4.8,1.2], [0,-1.8,-1.8,-1.8], s=100, c='#000000', marker='x')
# plt.scatter([3.6], [0], s=100, c='#000000', marker='x')
# plt.title('Outdoors')

# plt.axis('equal')
# plt.title('Jackal repeat run: odometry vs SLAM trajectory')
plt.show()