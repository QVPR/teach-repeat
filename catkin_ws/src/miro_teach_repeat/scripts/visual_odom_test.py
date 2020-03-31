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
	image_files = [dir+f for f in os.listdir(dir) if f[-10:] == '_image.pkl']
	image_files.sort()
	return image_files

def read_pose_files(pose_files):
	return [tf_conversions.fromMsg(message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',json.loads(read_file(p)))) for p in pose_files]
	
def read_image_files(image_files):
	return [pickle.loads(read_file(img)) for img in image_files]

def get_pose_x_y_theta(poses):
	
	x = np.array([pose.p.x() for pose in poses])
	y = np.array([pose.p.y() for pose in poses])
	theta = np.array([pose.M.GetRPY()[2] for pose in poses])
	return x, y, theta

def wrapToPi(x):
	return ((x + math.pi) % (2*math.pi)) - math.pi

#### #### ####
base_dir = os.path.expanduser('~/miro/data/')
file_dir = base_dir + 'J-lino1/'

poses = read_pose_files(get_pose_files(file_dir))
images = read_image_files(get_image_files(file_dir))
pose_data = get_pose_x_y_theta(poses)
visual_data = np.zeros((len(images),3))

visual_data[0][0] = pose_data[0][0]
visual_data[0][1] = pose_data[1][0]
visual_data[0][2] = pose_data[2][0]

for i in range(1,len(visual_data)):
	prev_img = images[i-1][5:20,30:-30]
	current_img = images[i][5:20,30:-30]
	offset = image_processing.image_scanline_rotation(prev_img, current_img)[1]
	gain = -0.03
	dtheta = gain * offset

	dd = (poses[i].p - poses[i-1].p).Norm()
	dx = dd * math.cos(visual_data[i-1,2])
	dy = dd * math.sin(visual_data[i-1,2])

	visual_data[i] = visual_data[i-1] + [dx,dy,dtheta]

f1 = plt.figure(1)
plt.plot(pose_data[2], c='#00ff00')
plt.plot(visual_data[:,2], c='#0000ff')

f2 = plt.figure(2)
plt.quiver(pose_data[0],pose_data[1],np.cos(pose_data[2]),np.sin(pose_data[2]), scale=50, color='#00ff00')
plt.quiver(visual_data[:,0],visual_data[:,1],np.cos(visual_data[:,2]),np.sin(visual_data[:,2]), scale=50, color='#0000ff')

# plt.scatter([3.5], [0], s=100, c='#000000', marker='x')
# plt.scatter([3.5,3.5], [0,-2], s=100, c='#000000', marker='x')
plt.scatter([3.5,3.5,1.15], [0,-2,-0.8], s=100, c='#000000', marker='x')

plt.show()
