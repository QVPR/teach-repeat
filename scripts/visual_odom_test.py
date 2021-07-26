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
import tf_conversions

import importlib
sys.path.append(os.path.dirname(__file__) + '/../nodes')
image_processing = importlib.import_module('image_processing')

def read_file(filename):
	with open(filename, 'r') as f:
		data = f.read()
	return data

def get_pose_files(dir):
	pose_files = [dir+f for f in os.listdir(dir) if f[-9:] == '_pose.txt']
	pose_files.sort()
	return pose_files

def get_image_files(dir):
	image_files = [dir+f for f in os.listdir(dir) if f[-9:] == '_full.png']
	image_files.sort()
	return image_files

def read_pose_files(pose_files):
	return [tf_conversions.fromMsg(message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',json.loads(read_file(p)))) for p in pose_files]
	
def read_image_files(image_files):
	return [cv2.imread(img) for img in image_files]

def get_pose_x_y_theta(poses):
	
	x = np.array([pose.p.x() for pose in poses])
	y = np.array([pose.p.y() for pose in poses])
	theta = np.array([pose.M.GetRPY()[2] for pose in poses])
	return x, y, theta

def wrapToPi(x):
	return ((x + math.pi) % (2*math.pi)) - math.pi

def unwrapFromPi(x):
	diff = x[1:] - x[:-1]
	for i in np.where(diff > math.pi)[0]:
		x[i+1:] -= 2*math.pi
	for i in np.where(diff < -math.pi)[0]:
		x[i+1:] += 2*math.pi
	return x

GAIN = -3.0 # gain per image width (pixels)
DEBUG = False

def integrate_visual_odometry(images, poses):
	visual_data = np.zeros((len(images),3))
	visual_data[0][0] = poses[0].p.x()
	visual_data[0][1] = poses[0].p.y()
	visual_data[0][2] = poses[0].M.GetRPY()[2]

	for i in range(1,len(visual_data)):
		prev_img = image_processing.grayscale(images[i-1][180:260,280:-280])
		current_img = image_processing.grayscale(images[i][180:260,280:-280])
		offset = image_processing.image_patch_rotation(prev_img, current_img, prev_img.shape[1]//2)[0]

		if DEBUG:
			debug_image = np.vstack((prev_img,current_img))
			cv2.imshow('img',debug_image)
			print('offset = ' + str(offset))
			cv2.waitKey()

		gain = GAIN / images[i].shape[1]
		dtheta = gain * offset

		dd = (poses[i].p - poses[i-1].p).Norm()
		dx = dd * math.cos(visual_data[i-1,2])
		dy = dd * math.sin(visual_data[i-1,2])

		visual_data[i] = visual_data[i-1] + [dx,dy,dtheta]
	return visual_data


#### #### ####
base_dir = os.path.expanduser('~/miro/data/')
dir_name = 'home'
file_dirs = [base_dir + dir_name + str(i+1) + '/' for i in range(1)]

poses_list = [read_pose_files(get_pose_files(file_dir)) for file_dir in file_dirs]
images_list = [read_image_files(get_image_files(file_dir)) for file_dir in file_dirs]
pose_data_list = [get_pose_x_y_theta(poses) for poses in poses_list]
visual_data_list = [integrate_visual_odometry(images, poses) for (images,poses) in zip(images_list,poses_list)]

colours = ['#00ff00', '#0000ff', '#ff0000']

f1 = plt.figure(1)

for i,(pose_data,visual_data,colour) in enumerate(zip(pose_data_list,visual_data_list,colours)):
	plt.subplot(len(file_dirs),1,i+1)
	plt.plot(unwrapFromPi(pose_data[2]), c=colour, alpha=0.5, label='odom')
	plt.plot(unwrapFromPi(visual_data[:,2]), '--', c=colour, label='visual odom')
plt.subplot(len(file_dirs),1,1)
plt.legend(loc='upper center', bbox_to_anchor=(0.5,1.5), ncol=2)

f = plt.figure(2)
for pose_data,visual_data,colour in zip(pose_data_list,visual_data_list,colours):
	plt.subplot(1,2,1)
	plt.quiver(pose_data[0],pose_data[1],np.cos(pose_data[2]),np.sin(pose_data[2]), scale=50, color=colour)
	plt.subplot(1,2,2)
	plt.quiver(visual_data[:,0],visual_data[:,1],np.cos(visual_data[:,2]),np.sin(visual_data[:,2]), scale=50, color=colour)

plt.subplot(1,2,1)
plt.axis('equal')
plt.title('odom')
# plt.scatter([3.5], [0], s=100, c='#000000', marker='x')
# plt.scatter([3.5,3.5], [0,-2], s=100, c='#000000', marker='x')
# plt.scatter([3.5,3.5,1.15], [0,-2,-0.8], s=100, c='#000000', marker='x')
plt.subplot(1,2,2)
plt.axis('equal')
plt.title('visual odom')
# plt.scatter([3.5], [0], s=100, c='#000000', marker='x')
# plt.scatter([3.5,3.5], [0,-2], s=100, c='#000000', marker='x')
# plt.scatter([3.5,3.5,1.15], [0,-2,-0.8], s=100, c='#000000', marker='x')


plt.show()
