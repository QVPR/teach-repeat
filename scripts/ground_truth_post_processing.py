import os
import json
import math
import numpy as np
from rospy_message_converter import message_converter
from geometry_msgs.msg import Pose
import tf_conversions
import matplotlib.pyplot as plt

def read_file(filename):
	with open(filename, 'r') as f:
		data = f.read()
	return data

def get_ground_truth_poses(dir):
	pose_files = np.array([f for f in os.listdir(dir) if f.endswith('_map_to_base_link.txt')])
	nums = [int(s.split('_')[0]) for s in pose_files]
	idx = np.argsort(nums)
	pose_files = [dir+f for f in pose_files[idx]]
	transforms = [message_converter.convert_dictionary_to_ros_message('geometry_msgs/TransformStamped',json.loads(read_file(p))).transform for p in pose_files]
	return [tf_conversions.fromMsg(Pose(tf.translation,tf.rotation)) for tf in transforms]

def get_pose_x_y_theta(poses):
	x = np.array([pose.p.x() for pose in poses])
	y = np.array([pose.p.y() for pose in poses])
	theta = np.array([pose.M.GetRPY()[2] for pose in poses])
	return x, y, theta

def pad_frame_to_length(poses, length):
	'''Chuck some nans at the start and on the end so the runs line up.'''
	poses = [None] * (length - len(poses)) + poses + [None]
	return poses

def pad_pose_x_y_to_length(poses, length):
	'''Chuck some nans at the start and on the end so the runs line up.'''
	x = np.hstack((np.full((length - len(poses[0])), np.nan), poses[0], np.full((1), np.nan)))
	y = np.hstack((np.full((length - len(poses[1])), np.nan), poses[1], np.full((1), np.nan)))
	theta = np.hstack((np.full((length - len(poses[2])), np.nan), poses[2], np.full((1), np.nan)))
	return x, y, theta

def wrapToPi(x):
	'''wrap angle to between +pi and -pi'''
	return ((x + math.pi) % (2*math.pi)) - math.pi

def quiver_plot(x_y_thetas, colour):
	plt.quiver(x_y_thetas[0], x_y_thetas[1], np.cos(x_y_thetas[2]), np.sin(x_y_thetas[2]), color=colour, scale=50)

def line_plot(x_y_thetas, colour):
	plt.plot(x_y_thetas[0], x_y_thetas[1], color=colour)

teach_run = '/home/dominic/Desktop/bearnav-test/'
repeat_runs = ['/home/dominic/Desktop/bearnav-test-repeat1/','/home/dominic/Desktop/pose/','/home/dominic/Desktop/pose-filtered/']

teach_poses = get_ground_truth_poses(teach_run)
repeat_poses = [get_ground_truth_poses(repeat_dir) for repeat_dir in repeat_runs]

teach_x_y_thetas = get_pose_x_y_theta(teach_poses)
repeat_x_y_thetas = [get_pose_x_y_theta(repeat_pose) for repeat_pose in repeat_poses]


colours = ['#44dd44', '#4444dd', '#dd4444', '#dddd44']

line_plot(teach_x_y_thetas, colours[0])

for pose_data_list,colour in zip(repeat_x_y_thetas,colours[1:]):
	line_plot(pose_data_list, colour)

plt.legend(['teach','ours filtered odom','bearnav filtered odom','bearnav unfiltered odom'])

# offsets = [[teach_pose.Inverse() * repeat_pose if repeat_pose is not None else None for teach_pose, repeat_pose in zip(teach_poses, repeat_pose_list)] for repeat_pose_list in repeat_poses]

# for i in range(len(repeat_poses)):
# 	repeat_poses[i] = pad_frame_to_length(repeat_poses[i], len(teach_poses)-1)
# 	repeat_x_y_thetas[i] = pad_pose_x_y_to_length(repeat_x_y_thetas[i], len(teach_poses)-1)

# fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
# for pose_data_list,colour in zip(repeat_poses,colours):
# 	ax1.plot(pose_data_list[0]-teach_poses[0], color=colour)
# 	ax2.plot(pose_data_list[1]-teach_poses[1], color=colour)
# 	ax3.plot(180/math.pi*(wrapToPi(pose_data_list[2]-teach_poses[2])), color=colour)


plt.show()