#!/usr/bin/env python

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

import sys
import importlib
sys.path.append(os.path.dirname(__file__) + '/../nodes')
image_processing = importlib.import_module('image_processing')

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

def load_images_cv(directory, normalise=True, convert_to_grayscale=False):
	image_files = get_sorted_files_by_ending(directory, '.png')
	if convert_to_grayscale:
		images = [image_processing.grayscale(cv2.imread(image)) for image in image_files]
	else:
		images = [cv2.imread(image, cv2.IMREAD_GRAYSCALE) for image in image_files]
	if normalise:
		return np.array([normalise_image(image) for image in images])
	else:
		return np.array(images)

def normalise_image(image):
	return np.float64(2.0*image/255.0 - 1.0)

def unnormalise_image(image):
	return np.uint8(255.0*(1+image)/2.0)

def get_image_indices_cv(directory):
	image_files = get_sorted_files_by_ending(directory, '.png')
	return np.array([int(os.path.basename(f)[:-4]) for f in image_files])

def get_image_keyframes(images, debug_images):
	img_shape = images[0].shape
	not_dupes = [True] + [not np.allclose(debug_images[i],debug_images[i-1]) for i in range(1,len(debug_images))]
	deduped = debug_images[not_dupes]
	new_goal = [False] + [not np.allclose(deduped[i,img_shape[0]:2*img_shape[0],:], deduped[i-1,img_shape[0]:2*img_shape[0],:]) for i in range(1,len(deduped))]
	debug_keyframes = deduped[new_goal,:img_shape[0],:]
	keyframe_indices = [0] + [np.argmin([np.sum(np.abs(keyframe[:,:img_shape[1]]-test_image)) for test_image in images]) for keyframe in debug_keyframes]
	keyframes = np.array(images[keyframe_indices])
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

def load_corr_data(directory):
	correlation_data_files = get_sorted_files_by_ending(directory, '.txt')
	correlation_data = [read_file(f) for f in correlation_data_files]
	pos = [float(corr_data.split('\n')[0]) for corr_data in correlation_data]
	u = [float(corr_data.split('\n')[1]) for corr_data in correlation_data]
	corr = [[float(num) for num in corr_data.split('\n')[2][1:-1].split(' ') if num != ''] for corr_data in correlation_data]
	return pos, u, corr

def get_confusion_matrix(directory, reference_images, test_images):
	if not os.path.exists(directory + 'confusion.npy'):
		correlations, offsets = confusion_matrix.confusion_matrix(reference_images, test_images)
		np.save(directory + 'confusion', correlations)
		np.save(directory + 'offsets', offsets)
	else:
		correlations = np.load(dir_test + 'confusion.npy')
		offsets = np.load(dir_test + 'offsets.npy')
	return correlations, offsets

def plot_along_route_localisation(correlations, path_offsets, correspondances):
	fig, (ax1, ax2) = plt.subplots(2, 1, gridspec_kw={'height_ratios': (5,1)}, sharex=True)

	# scale path_offsets to between 0 and 1
	corrected_path_offsets = path_offsets - 1.0
	corrected_path_offsets[corrected_path_offsets < 0] = -1.0/(corrected_path_offsets[corrected_path_offsets < 0] + 1)
	corrected_path_offsets[corrected_path_offsets > 0] += 1.0
	max_diff = np.abs(corrected_path_offsets).max()
	corrected_path_offsets[corrected_path_offsets < 0] += 1.0
	corrected_path_offsets[corrected_path_offsets > 0] -= 1.0
	corrected_path_offsets /= (2 * max_diff)
	corrected_path_offsets += 0.5
	colours = plt.cm.coolwarm(corrected_path_offsets)

	ax1.scatter(np.arange(correlations.shape[1]), np.argmax(correlations, axis=0), color='black', marker='.', alpha=0.4)
	ax1.scatter(correspondances, np.arange(len(correspondances)) % correlations.shape[0], marker='x', color=colours)
	sm = matplotlib.cm.ScalarMappable(cmap=plt.cm.coolwarm)
	sm.set_array([])
	cb = fig.colorbar(sm, ax=ax1, orientation='horizontal', pad=0.05, aspect=50)
	# cb.set_label('along-path correction')
	cb.set_ticks([0,0.5,1])
	cb.ax.set_xticklabels(['$\\div$%.2f (reduce)' % (max_diff),'$\\times$1.0','$\\times$%.2f (extend)' % (max_diff)])
	ax1.set_ylabel('keyframe number')
	# ax1.set_xticklabels([])

	ax2.axhline(1.0, linestyle='--', color='grey')
	ax2.plot(correspondances, path_offsets[:len(correspondances)])
	ax2.set_xlabel('image frame number')
	ax2.set_ylabel('path correction')
	ax2.set_yscale('log')
	plt.tight_layout()

def plot_odom_theta_offset(poses, theta_offsets, reference_poses):
	fig, (ax1, ax2) = plt.subplots(2, 1, gridspec_kw={'height_ratios': (5,1)})

	theta_lim = math.ceil(math.degrees(np.abs(theta_offsets).max()))
	corrected_theta_offsets = 0.5 + 0.5*(theta_offsets / math.radians(theta_lim)).clip(min=-1, max=1)
	colours = plt.cm.coolwarm(corrected_theta_offsets)

	ax1.quiver(reference_poses.x, reference_poses.y, np.cos(reference_poses.theta), np.sin(reference_poses.theta), color='black', scale=50)
	ax1.quiver(poses.x, poses.y, np.cos(poses.theta), np.sin(poses.theta), color=colours, scale=50)
	sm = matplotlib.cm.ScalarMappable(cmap=plt.cm.coolwarm)
	sm.set_array([-theta_lim, theta_lim])
	cb = fig.colorbar(sm, ax=ax1)
	cb.set_label(r'angle correction, $\Delta\theta$ ($^\circ$)')
	ax1.axis('equal')

	ax2.axhline(0.0, linestyle='--', color='grey')
	ax2.plot(np.degrees(theta_offsets))
	ax2.set_ylabel(r'$\Delta\theta$ ($^\circ$)')
	ax2.set_xticklabels([])
	plt.tight_layout()

def plot_odom_path_offsets(poses, path_offsets, reference_poses):
	fig, (ax1, ax2) = plt.subplots(2, 1, gridspec_kw={'height_ratios': (5,1)})

	max_extension = path_offsets.max()
	max_contraction = (1./path_offsets).max()
	path_lim = max(max_extension, max_contraction)
	corrected_path_offsets = path_offsets.copy()
	corrected_path_offsets[corrected_path_offsets < 1] = -1./corrected_path_offsets[corrected_path_offsets < 1]
	corrected_path_offsets = 0.5 + corrected_path_offsets/(2*path_lim)
	colours = plt.cm.coolwarm(corrected_path_offsets)

	ax1.quiver(reference_poses.x, reference_poses.y, np.cos(reference_poses.theta), np.sin(reference_poses.theta), color='black', scale=50)
	ax1.quiver(poses.x, poses.y, np.cos(poses.theta), np.sin(poses.theta), color=colours, scale=50)
	sm = matplotlib.cm.ScalarMappable(cmap=plt.cm.coolwarm)
	sm.set_array([])
	cb = fig.colorbar(sm, ax=ax1)
	cb.set_label(r'path correction')
	cb.set_ticks([0,0.5,1])
	cb.ax.set_yticklabels(['$\\div$%.2f (reduce)' % (path_lim),'$\\times$1.0','$\\times$%.2f (extend)' % (path_lim)])
	ax1.axis('equal')

	ax2.axhline(1.0, linestyle='--', color='grey')
	ax2.plot(path_offsets)
	ax2.set_ylabel('Path correction')
	ax2.set_xticklabels([])
	plt.tight_layout()

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
	correspondance_full = get_full_correspondances(correspondances, correlations.shape[1])
	
	if correspondances[-1] != correlations.shape[1]:
		correspondances = np.hstack((correspondances,[correlations.shape[1]]))

	diffs = [a - b for a,b in zip(correspondances[1:], correspondances[:-1])]
	u = np.array([a for n in diffs for a in np.arange(-.5,.5,1./n)])

	corr_range = np.arange(-search_range, search_range+2)-.5
	corr_data = np.zeros((len(corr_range), correspondance_full.size))
	indices = np.int32(np.tile(correspondance_full, (len(corr_range),1)) + (corr_range - 0.5).reshape(-1,1))
	valid_indices = (indices >= 0) & (indices < correlations.shape[0])
	for i in range(indices.shape[1]):
		corr_data[valid_indices[:,i],i] = correlations[indices[valid_indices[:,i],i],i]

	corr_data -= 0.1
	corr_data[corr_data < 0] = 0.0
	with np.errstate(divide='ignore', invalid='ignore'):
		corr_data /= corr_data.sum(axis=0)
	corr_data[np.isnan(corr_data)] = 0.0

	weighted_average = np.sum(corr_data * corr_range.reshape(-1,1), axis=0)

	n = 5
	corr_data = np.repeat(corr_data, n, axis=0)

	# Nbins = 16
	# bins = np.digitize(u, np.arange(-0.5, 0.6, 1./Nbins))-1
	# avg_dist = [corr_data[:,i == bins].mean(axis=1) for i in range(Nbins)]
	# avg = [np.sum(d * corr_range) for d in avg_dist]
	# avg2 = np.zeros(len(avg))

	# fig, axs = plt.subplots(int(math.sqrt(Nbins)), int(math.sqrt(Nbins)), sharex=True, sharey=True)
	# for i in range(Nbins):
	# 	ax = axs[int(i/math.sqrt(Nbins)),int(i%math.sqrt(Nbins))]
	# 	ax.plot(corr_range, avg_dist[i])
		# y_32n = avg_dist[i][corr_range == -1.5]
		# # y_12n = avg_dist[i][corr_range == -0.5]
		# y_12p = avg_dist[i][corr_range == 0.5]
		# y_32p = avg_dist[i][corr_range == 1.5]
		# b = 1./3. / (1./3 + math.log(y_32p/y_12p) / math.log(y_32n/y_32p))
		# # b = math.log(y_12n/y_12p) / (math.log(y_32p*y_12n/y_12p**2))
		# k = -math.log(y_32n/y_32p) / 6. / b
		# A = y_12p / math.exp(-k*(b**2 - b + 1./4))
		# dense_range = np.arange(-search_range, search_range+1.1, 0.1)-.5
		# ax.plot(dense_range, A*np.exp(-k * (dense_range - b)**2), '--')
		# avg2[i] = b

	# plt.figure()
	# plt.plot(np.arange(-0.5+0.5/Nbins, 0.5, 1./Nbins), avg)
	# plt.plot(np.arange(-0.5+0.5/Nbins, 0.5, 1./Nbins), avg2)

	# plt.figure()
	# # plt.plot(weighted_average[correspondance_full == 10])
	# x = 15
	# plt.plot(corr_data[:,correspondance_full == x] + 0.0*np.arange(np.sum(correspondance_full == x)))
	# plt.figure()
	# plt.plot(u[correspondance_full == x], weighted_average[correspondance_full == x])

	plt.figure()
	plt.imshow(corr_data[:,:], cmap='viridis')
	plt.clim([0,1])
	plt.plot(n*(weighted_average + search_range + 1)-0.5, 'r.')
	# plt.figure()
	# plt.plot(correspondance_full + u + 0.5, correspondance_full + weighted_average + 0.5)
	# plt.figure()
	# plt.scatter(u, weighted_average)
	# print(np.corrcoef(u, weighted_average))

def get_full_correspondances(correspdance_indices, length):
	if correspdance_indices[0] != 0:
		correspdance_indices = np.hstack(([0],correspdance_indices))
	correspondance_full = np.zeros(length, dtype=np.uint32)
	correspondance_full[correspdance_indices] = 1
	correspondance_full = np.cumsum(correspondance_full)
	return correspondance_full

if __name__ == "__main__":
	dir_reference = os.path.expanduser('~/miro/data/office5/')
	dir_test = os.path.expanduser('~/miro/data/office5_tests/20/')

	reference_images = load_images(dir_reference)
	reference_images_full = load_images_cv(dir_reference+'full/', normalise=False)
	reference_poses = load_poses(dir_reference)

	# test_keyframes = load_images(dir_test)
	# test_images = load_images_cv(dir_test + 'norm/')
	# test_images_full = load_images_cv(dir_test + 'full/', normalise=False)
	# if reference_images[0].shape != test_images[0].shape:
	# 	reference_images = [cv2.resize(img, tuple(reversed(test_images[0].shape)), interpolation=cv2.INTER_AREA) for img in reference_images]

	# test_keyframes = get_image_keyframes(load_images_cv(dir_test + 'norm/'), load_images_cv(dir_test))
	# test_image_indices = get_image_indices_cv(dir_test + 'norm/')
	test_poses = load_poses(dir_test + 'pose/')
	test_offsets = load_poses(dir_test + 'offset/')
	test_corrections = load_corrections(dir_test + 'correction/')
	pos, u, corr = load_corr_data(dir_test)
	# test_keyframe_correspondances = np.array([np.argmin([np.sum(np.abs(keyframe-test_image)) for test_image in test_images]) for keyframe in test_keyframes])

	# fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
	# ax1.plot(u)
	# ax2.plot(pos)
	# from scipy.stats import linregress
	# plt.scatter(u, pos)

	# print(linregress(u, pos))

	max_row_length = max([len(r) for r in corr])
	for i, row in enumerate(corr):
		if len(row) < max_row_length:
			row = [np.nan] * ((max_row_length - len(row))/2) + row + [np.nan] * ((max_row_length - len(row))/2)
			corr[i] = row
	corr = np.array(corr)

	corr_edit = corr.copy()
	corr_edit -= 0.1
	corr_edit[corr_edit < 0] = 0
	corr_edit /= corr_edit.sum(axis=1).reshape(-1,1)
	corr_pos = np.sum(corr_edit * np.arange(-2.5,2.6,1).reshape(1,-1), axis=1)
	# corr_pos -= np.array(u)
	print(plt.ylim())

	plt.imshow(np.rot90(corr), aspect='auto', origin='lower')
	plt.plot(np.array(pos) + 2.5, 'r')
	plt.plot(corr_pos + 2.5, 'm')
	plt.plot(np.array(u) + 2.)

	# correlations, offsets = get_confusion_matrix(dir_test, reference_images, test_images)

	# corr_at_keyframes = correlations[np.arange(len(test_keyframe_correspondances)) % correlations.shape[0],test_keyframe_correspondances]
	
	# plt.imshow(correlations)

	# plot_along_route_localisation(correlations, test_corrections.path_offset, test_keyframe_correspondances)

	# plot_image_along_path_localisation_full(correlations, test_keyframe_correspondances, 4)

	# plot_odom_theta_offset(test_poses, test_corrections.theta_offset, reference_poses)
	# plot_odom_path_offsets(test_poses, test_corrections.path_offset, reference_poses)

	# plt.imshow(offsets)

	# plt.figure()
	# plt.plot(offsets[get_full_correspondances(test_keyframe_correspondances, offsets.shape[1]),np.arange(offsets.shape[1])])

	def get_mean(x):
		y = x# - 0.1
		y[y < 0] = 0.0
		y /= y.sum()
		return np.sum(y * np.arange(x.size))

	def get_quadratic_mean(x):
		poly = np.poly1d(np.polyfit(np.arange(x.size), x, 2))
		return -poly.coefficients[1] / 2 / poly.coefficients[0]

	def plot_with_mean(x):
		plt.plot(x)
		# plt.plot(get_mean(x), x.max(),'x')
		# poly = np.poly1d(np.polyfit(np.arange(x.size), x, 2))
		# plt.plot(np.arange(0,x.size-1,0.1),poly(np.arange(0,x.size-1,0.1)))
		# peak = -poly.coefficients[1] / 2 / poly.coefficients[0]
		# plt.plot(peak, poly(peak),'x')

	# plot_with_mean(correlations[13:17,236])
	# plot_with_mean(correlations[13:17,248])
	# plt.figure()
	# for i in range(236,249):
	# 	plot_with_mean(i*0.0 + correlations[13:17,i])

	# plt.plot([get_mean(correlations[13:17,i]) for i in range(236,249)])
	# plt.plot([get_quadratic_mean(correlations[13:17,i]) for i in range(236,249)])

	plt.show()
	