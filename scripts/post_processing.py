#!/usr/bin/python

import os
import pickle
import json
import math
import numpy as np
import cv2

from rospy_message_converter import message_converter
import tf_conversions
import matplotlib.pyplot as plt
import matplotlib.style

import confusion_matrix

class Poses2D(object):
	def __init__(self, x, y, theta, frames):
		self.x = x
		self.y = y
		self.theta = theta
		self.frames = frames

class Corrections(object):
	def __init__(self, path_offset, theta_offset):
		self.path_offset = path_offset
		self.theta_offset = theta_offset	

def read_file(filename):
	with open(filename, 'r') as f:
		data = f.read()
	return data

def get_sorted_files_by_ending(directory, file_ending):
	files = [directory+f for f in os.listdir(directory) if f[-len(file_ending):] == file_ending]
	files.sort()
	return files

def load_images(directory):
	image_files = get_sorted_files_by_ending(directory, '_image.pkl')
	return np.array([pickle.loads(read_file(image)) for image in image_files])

def load_images_cv(directory):
	image_files = get_sorted_files_by_ending(directory, '.png')
	return np.array([normalise_image(cv2.imread(image, cv2.IMREAD_GRAYSCALE)) for image in image_files])

def normalise_image(image):
	return np.float64(2.0*image/255.0 - 1.0)

def unnormalise_image(image):
	return np.uint8(255.0*(1+image)/2.0)

def get_image_indices_cv(directory):
	image_files = get_sorted_files_by_ending(directory, '.png')
	return np.array([int(os.path.basename(f)[:-4]) for f in image_files])

def load_poses(directory):
	pose_files = get_sorted_files_by_ending(directory, '_pose.txt')
	return poses_to_np([tf_conversions.fromMsg(message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',json.loads(read_file(p)))) for p in pose_files])

def poses_to_np(poses):
	return Poses2D(np.array([pose.p.x() for pose in poses]),np.array([pose.p.y() for pose in poses]),np.array([pose.M.GetRPY()[2] for pose in poses]), np.array(poses))

def load_corrections(directory):
	pose_files = get_sorted_files_by_ending(directory, '_correction.txt')
	corrections = [json.loads(read_file(f)) for f in pose_files]
	# append default values so the corrections correspond to the corrected goals
	# (not the previous goal, where the correction was calculated)
	path_offets = np.array([1.0] + [correction['path_offset'] for correction in corrections])
	theta_offsets = np.array([0.0] + [correction['theta_offset'] for correction in corrections])
	return Corrections(path_offets, theta_offsets)

def get_confusion_matrix(directory, reference_images, test_images):
	if not os.path.exists(directory + 'confusion.npy'):
		correlations, offsets = confusion_matrix.confusion_matrix(reference_images, test_images)
		np.save(directory + 'confusion', correlations)
		np.save(directory + 'offsets', offsets)
	else:
		correlations = np.load(dir_test + 'confusion.npy')
		offsets = np.load(dir_test + 'offsets.npy')
	return correlations, offsets

def plot_along_route_localisation(correlations, path_offset, correspondances):
	fig, ax = plt.subplots()

	corrected_path_offsets = path_offset
	corrected_path_offsets[corrected_path_offsets == 1.0] = 0.0
	corrected_path_offsets[(corrected_path_offsets < 1.0) & (corrected_path_offsets > 0.0)] = -np.log(corrected_path_offsets[(corrected_path_offsets < 1.0) & (corrected_path_offsets > 0.0)])/math.log(0.5)
	corrected_path_offsets[corrected_path_offsets > 1.0] = np.log(corrected_path_offsets[corrected_path_offsets > 1.0])/math.log(1.5)
	corrected_path_offsets /= 2*2
	corrected_path_offsets += 0.5
	colours = plt.cm.coolwarm(corrected_path_offsets)

	ax.scatter(np.arange(correlations.shape[1]), np.argmax(correlations, axis=0), color='black', marker='.', alpha=0.4)
	ax.scatter(correspondances, np.arange(len(correspondances)), marker='x', color=colours)

	sm = matplotlib.cm.ScalarMappable(cmap=plt.cm.coolwarm)
	sm.set_array([])
	cb = fig.colorbar(sm)
	cb.set_label('along-path correction')
	cb.set_ticks([0,0.5,1])
	cb.ax.set_yticklabels(['reduce','none','extend'])
	plt.xlabel('image frame number')
	plt.ylabel('keyframe number')

def plot_odom_theta_offset(correlations, poses, theta_offsets):
	# plt.style.use('ggplot')
	fig, ax = plt.subplots()

	theta_lim = 15
	corrected_theta_offsets = 0.5 + 0.5*(theta_offsets / math.radians(theta_lim)).clip(min=-1, max=1)
	colours = plt.cm.coolwarm(corrected_theta_offsets)

	ax.quiver(poses.x, poses.y, np.cos(poses.theta), np.sin(poses.theta), color=colours, scale=50)

	sm = matplotlib.cm.ScalarMappable(cmap=plt.cm.coolwarm)
	sm.set_array([-theta_lim, theta_lim])
	cb = fig.colorbar(sm)
	cb.set_label('theta correction (degrees)')

def integrate_corrected_poses(poses, origin, theta_corrections, path_corrections):
	corrected_poses = [poses.frames[0]]

	for i in range(1,len(poses.frames)):
		new_pose = poses.frames[i]
		old_pose = poses.frames[i-1]
		current_goal = corrected_poses[-1]

		pose_offset = old_pose.Inverse() * new_pose
		pose_offset.p = tf_conversions.Rotation.RotZ(current_goal.M.GetRPY()[2]) * pose_offset.p
		pose_offset_corrected = pose_offset
		if theta_corrections is not None and len(theta_corrections) > i:
			pose_offset_corrected = tf_conversions.Frame(tf_conversions.Rotation.RotZ(theta_corrections[i])) * pose_offset_corrected
		if path_corrections is not None and len(path_corrections) > i:
			pose_offset_corrected.p *= path_corrections[i]
		corrected_poses.append(tf_conversions.Frame(pose_offset_corrected.M * current_goal.M, pose_offset_corrected.p + current_goal.p))

	origin_offset = poses.frames[0].Inverse() * origin
	corrected_poses = [pose * origin_offset for pose in corrected_poses]

	return poses_to_np(corrected_poses)

if __name__ == "__main__":
	dir_reference = os.path.expanduser('~/miro/data/odom-breaking/')
	dir_test = os.path.expanduser('~/miro/data/odom-breaking_tests2/42/')

	reference_images = load_images(dir_reference)
	reference_poses = load_poses(dir_reference)

	test_keyframes = load_images(dir_test)
	test_images = load_images_cv(dir_test + 'norm/')
	test_image_indices = get_image_indices_cv(dir_test + 'norm/')
	test_poses = load_poses(dir_test + 'pose/')
	test_offsets = load_poses(dir_test + 'offset/')
	test_corrections = load_corrections(dir_test + 'correction/')
	test_keyframe_correspondances = np.array([np.argmin([np.sum(np.abs(keyframe-test_image)) for test_image in test_images]) for keyframe in test_keyframes])

	correlations, offsets = get_confusion_matrix(dir_test, reference_images, test_images)

	corr_at_keyframes = correlations[np.arange(len(test_keyframe_correspondances)),test_keyframe_correspondances]

	plot_along_route_localisation(correlations, test_corrections.path_offset, test_keyframe_correspondances)

	plt.show()
	