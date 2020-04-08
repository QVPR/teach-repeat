#!/usr/bin/python

import numpy as np
import cv2
import os
import pickle
import time
import json
import math
import sys
import tf_conversions
from rospy_message_converter import message_converter
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(__file__) + '/../nodes')
import importlib
image_processing = importlib.import_module('image_processing')


def read_file(filename):
	with open(filename, 'r') as f:
		data = f.read()
	return data

def read_pose_files(pose_files):
	return [tf_conversions.fromMsg(message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',json.loads(read_file(p)))) for p in pose_files]
	
def read_image_files(image_files):
	return [pickle.loads(read_file(img)) for img in image_files]

dir = os.path.expanduser('~/miro/data/image-test/')

files_n = range(4)

images = read_image_files([dir + ("%06d_image.pkl" % i) for i in files_n])
poses = read_pose_files([dir + ("%06d_pose.txt" % i) for i in files_n])

thetas = np.zeros(len(files_n)-1)
offsets = np.zeros(len(files_n)-1)
correlations = np.zeros(len(files_n)-1)

for i,(image,pose) in enumerate(zip(images[1:],poses[1:])):
	offset, correlation = image_processing.xcorr_match_images(images[0], image)
	offsets[i] = offset
	correlations[i] = correlation
	thetas[i] = math.degrees(pose.M.GetRPY()[2])

plt.scatter(thetas, offsets)
plt.xlabel(r'offset $\theta$ ($\degree$)')
plt.ylabel('image offset (px)')
plt.title('horizontal offset = 0')

plt.show()