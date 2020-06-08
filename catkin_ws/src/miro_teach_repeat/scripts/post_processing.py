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

def get_image_keyframes(images, debug_images):
	not_dupes = [True] + [not np.allclose(debug_images[i],debug_images[i-1]) for i in range(1,len(debug_images))]
	deduped = debug_images[not_dupes]
	new_goal = [False] + [not np.allclose(deduped[i,44:88,:], deduped[i-1,44:88,:]) for i in range(1,len(deduped))]
	debug_keyframes = deduped[new_goal,:44,:]
	keyframes = np.array([images[np.argmin([np.sum(np.abs(keyframe-test_image)) for test_image in images])] for keyframe in debug_keyframes])
	return keyframes

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

def plot_image_along_path_localisation(correlations, correspondances, search_range):
	corr_range = np.arange(-search_range, search_range+1)
	corr_data = np.zeros((len(corr_range), correspondances.size))
	for i, corr in enumerate(corr_range):
		if corr < 0:
			corr = abs(corr)
			c = np.hstack((np.zeros(corr), correlations[np.arange(len(correspondances)-corr),correspondances[corr:]]))
		elif corr > 0:
			c = np.hstack((correlations[np.arange(corr,len(correspondances)),correspondances[:-corr]], np.zeros(corr)))
		else:
			c = correlations[np.arange(len(correspondances)),correspondances]
		corr_data[i,:] = c

	corr_data -= 0.1
	corr_data[corr_data < 0] = 0.0
	corr_data /= corr_data.sum(axis=0)

	weighted_average = np.sum(corr_data * corr_range.reshape(-1,1), axis=0)

	plt.figure()
	plt.imshow(corr_data[:,:], cmap='viridis')
	plt.clim([0,1])
	plt.plot(weighted_average + search_range, 'r.')

def plot_image_along_path_localisation_full(correlations, correspondances, search_range):
	correspondance_full = np.zeros(correlations.shape[1], dtype=np.uint32)
	correspondance_full[correspondances] = 1
	correspondance_full = np.cumsum(correspondance_full)

	diffs = [a - b for a,b in zip(list(correspondances)+[correlations.shape[1]], [0] + list(correspondances))]
	u = np.array([-a for n in diffs for a in np.arange(-.5,.5,1./n)])

	corr_range = np.arange(-search_range, search_range+2)-.5
	corr_data = np.zeros((len(corr_range), correspondance_full.size))
	for i, corr in enumerate(corr_range):
		corr = int(corr+0.5)
		if corr < 0:
			corr = abs(corr)
			c = np.hstack((np.zeros(corr), correlations[correspondance_full[corr:], np.arange(len(correspondance_full)-corr)]))
		elif corr > 0:
			c = np.hstack((correlations[correspondance_full[:-corr], np.arange(corr,len(correspondance_full))], np.zeros(corr)))
		else:
			c = correlations[correspondance_full, np.arange(len(correspondance_full))]
		corr_data[i,:] = c

	corr_data -= 0.1
	corr_data[corr_data < 0] = 0.0
	with np.errstate(divide='ignore', invalid='ignore'):
		corr_data /= corr_data.sum(axis=0)
	corr_data[np.isnan(corr_data)] = 0.0

	weighted_average = np.sum(corr_data * corr_range.reshape(-1,1), axis=0)

	n = 5
	corr_data = np.repeat(corr_data, n, axis=0)

	plt.figure()
	plt.imshow(corr_data[:,:], cmap='viridis')
	plt.clim([0,1])
	plt.plot(n*(weighted_average + search_range + 1)-0.5, 'r.')
	plt.figure()
	# plt.plot(u)
	plt.plot(weighted_average)

if __name__ == "__main__":
	dir_reference = os.path.expanduser('~/miro/data/follow-long-path/')
	dir_test = os.path.expanduser('~/miro/data/follow-long-path_tests/57/')

	reference_images = load_images(dir_reference)
	reference_poses = load_poses(dir_reference)

	# test_keyframes = load_images(dir_test)
	test_images = load_images_cv(dir_test + 'norm/')
	test_keyframes = get_image_keyframes(load_images_cv(dir_test + 'norm/'), load_images_cv(dir_test))
	test_image_indices = get_image_indices_cv(dir_test + 'norm/')
	test_poses = load_poses(dir_test + 'pose/')
	test_offsets = load_poses(dir_test + 'offset/')
	test_corrections = load_corrections(dir_test + 'correction/')
	test_keyframe_correspondances = np.array([np.argmin([np.sum(np.abs(keyframe-test_image)) for test_image in test_images]) for keyframe in test_keyframes])

	correlations, offsets = get_confusion_matrix(dir_test, reference_images, test_images)

	corr_at_keyframes = correlations[np.arange(len(test_keyframe_correspondances)),test_keyframe_correspondances]
	
	plot_along_route_localisation(correlations, test_corrections.path_offset, test_keyframe_correspondances)

	plot_image_along_path_localisation_full(correlations, test_keyframe_correspondances, 1)

	plt.show()
	