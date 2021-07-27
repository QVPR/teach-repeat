#!/usr/bin/env python

import numpy as np
import cv2
import os
import pickle
import time
import json
import yaml
import math
import sys
import matplotlib
import matplotlib.pyplot as plt
from rospy_message_converter import message_converter
import tf_conversions
import pandas as pd
import collections

import importlib
sys.path.append(os.path.dirname(__file__) + '/../nodes')
image_processing = importlib.import_module('image_processing')

THIS_DIR_PATH = os.path.dirname(os.path.abspath(__file__))

import post_processing

FULL_RES = 925.

def corr_images(img1, img2, subsample=1):
	img1_pad = np.pad(img1, ((0,),(int(img2.shape[1]/2),)), mode='constant', constant_values=0)
	return image_processing.normxcorr2_subpixel(img1_pad, img2, subsample)

def plot_corr(img1, img2, subsample=1, style='-', color=None):
	corr = corr_images(img1, img2, subsample)
	# t in px
	res = FULL_RES/img1.shape[1]/subsample
	# t = np.arange(-(len(corr)-1)/2*res, res*(len(corr)-1)/2+res/2, res)
	t = np.arange(-(len(corr)-1)/2, (len(corr)-1)/2+1)
	best = t[np.argmax(corr)]
	plt.plot(t, corr, style, color=color, label='[%dx%d]' % img1.shape[::-1])
	if subsample > 1:
		sparse_style = '.' if style == '-' else style+'.'
		plt.plot(t[0::subsample], corr[0::subsample], sparse_style, color=color)
		best_sparse = t[0::subsample][np.argmax(corr[0::subsample])]
		return corr, best, best_sparse
	else:
		return corr, best

def resize_images(imgs, size, interp=cv2.INTER_AREA):
	if type(size) == tuple:
		size_reversed = tuple(reversed(size))
		return [cv2.resize(img, size_reversed, interpolation=interp) for img in imgs]
	else:
		return [cv2.resize(img, None, fx=size, fy=size, interpolation=interp) for img in imgs]

def patch_normalise_images(imgs, patch_size=(9,9)):
	return [image_processing.patch_normalise_pad(img, patch_size) for img in imgs]

def load_camera_infos():
	info_left = image_processing.yaml_to_camera_info(yaml.load(image_processing.read_file(THIS_DIR_PATH + '../calibration/left_360.yaml'), Loader=yaml.SafeLoader))
	info_right = image_processing.yaml_to_camera_info(yaml.load(image_processing.read_file(THIS_DIR_PATH + '../calibration/right_360.yaml'), Loader=yaml.SafeLoader))
	return info_left, info_right

def test_downscaling(data_dir, sizes, rectify=False):
	images_left = post_processing.load_images_cv(data_dir + 'left/', convert_to_grayscale=True)
	images_right = post_processing.load_images_cv(data_dir + 'right/', convert_to_grayscale=True)
	thetas = post_processing.load_poses(data_dir).theta

	good_indices = abs(thetas) < math.radians(20)
	good_thetas = thetas[good_indices] * math.degrees(1)
	plt.plot(good_thetas, label='GT')

	if rectify:
		info_left, info_right = load_camera_infos()
		stitched_images = np.array([image_processing.rectify_stitch_stereo_image(image_left, image_right, info_left, info_right)[0] for image_left, image_right in zip(images_left, images_right)])
		fov = image_processing.rectify_stitch_stereo_image(images_left[0], images_right[0], info_left, info_right)[1]
	else:
		stitched_images = np.array([image_processing.stitch_stereo_image(image_left, image_right) for image_left, image_right in zip(images_left, images_right)])
		fov = 175.2

	rms_results = {'pixel':{}, 'subpixel':{}}
	for size in sizes:
		resized_images = patch_normalise_images(resize_images(stitched_images, 1./size))
		offsets = np.array([image_processing.xcorr_match_images(resized_images[0], image)[0] for image in resized_images])
		offsets_deg = -offsets[good_indices] * fov / resized_images[0].shape[1]
		plt.plot(offsets_deg, label=str(size))
		rms = np.sqrt(((offsets_deg - good_thetas)**2).mean())
		rms_results['pixel'][str(size)] = rms
		print('%d: RMS = %f' % (size, rms))

		if size != 1:
			offsets_subpixel = np.array([image_processing.xcorr_match_images(resized_images[0], image, size)[0] for image in resized_images])
			offsets_subpixel_deg = -offsets_subpixel[good_indices] * fov / resized_images[0].shape[1] / size
			plt.plot(offsets_subpixel_deg, '--', label=str(size)+'_sub')
			rms = np.sqrt(((offsets_subpixel_deg - good_thetas)**2).mean())
			rms_results['subpixel'][str(size)] = rms
			print('%d_sub: RMS = %f' % (size, rms))
	plt.legend()
	df = pd.DataFrame(rms_results)
	df = df.iloc[np.argsort([int(s) for s in list(df.index)])]
	df.plot(kind='bar')

def test_depth(data_dir, size, rectify=False):
	images_left = post_processing.load_images_cv(data_dir + 'left/', convert_to_grayscale=True)
	images_right = post_processing.load_images_cv(data_dir + 'right/', convert_to_grayscale=True)
	thetas = post_processing.load_poses(data_dir).theta
	depth_left = np.load(data_dir+'left/000000_disp.npy').squeeze()
	depth_right = np.load(data_dir+'right/000000_disp.npy').squeeze()

	good_indices = abs(thetas) < math.radians(20)
	good_thetas = thetas[good_indices] * math.degrees(1)
	plt.plot(good_thetas, label='GT')

	if rectify:
		info_left, info_right = load_camera_infos()
		stitched_images = np.array([image_processing.rectify_stitch_stereo_image(image_left, image_right, info_left, info_right)[0] for image_left, image_right in zip(images_left, images_right)])
		depth = image_processing.rectify_stitch_stereo_image(depth_left, depth_right, info_left, info_right)[0]
		fov = image_processing.rectify_stitch_stereo_image(images_left[0], images_right[0], info_left, info_right)[1]
	else:
		stitched_images = np.array([image_processing.stitch_stereo_image(image_left, image_right) for image_left, image_right in zip(images_left, images_right)])
		depth = image_processing.stitch_stereo_image(depth_left, depth_right)
		fov = 175.2

	rms_results = {'depth':0, 'no depth':0}
	# for size in sizes:
	resized_images = patch_normalise_images(resize_images(stitched_images, 1./size))
	depth = cv2.resize(depth, resized_images[0].shape[::-1], interpolation = cv2.INTER_CUBIC)
	offsets = np.array([image_processing.xcorr_match_images(resized_images[0], image)[0] for image in resized_images])
	offsets_deg = -offsets[good_indices] * fov / resized_images[0].shape[1]
	plt.plot(offsets_deg, label=str(size))
	rms = np.sqrt(((offsets_deg - good_thetas)**2).mean())
	rms_results['no depth'] = rms
	print('%d: RMS = %f' % (size, rms))

	# if size != 1:
	offsets_depth = np.array([image_processing.xcorr_match_images(depth*resized_images[0], image)[0] for image in resized_images])
	offsets_depth_deg = -offsets_depth[good_indices] * fov / resized_images[0].shape[1]
	plt.plot(offsets_depth_deg, '--', label=str(size)+' depth')
	rms = np.sqrt(((offsets_depth_deg - good_thetas)**2).mean())
	rms_results['depth'] = rms
	print('%d depth: RMS = %f' % (size, rms))
	plt.legend()
	df = pd.DataFrame(rms_results, index=[size])
	# df = df.iloc[np.argsort([int(s) for s in list(df.index)])]
	df.plot(kind='bar')


def test_full_path_offsets(data_dir):
	# if os.path.exists(test_dir + 'offset_data.npy'):
	# 	offsets_data = np.load(test_dir + 'offset_data.npy', allow_pickle=True)[()]
	# else:
	# 	offsets_data = {}
	info_left, info_right = load_camera_infos()
	images_left = post_processing.load_images_cv(data_dir + 'left/', convert_to_grayscale=True)
	images_right = post_processing.load_images_cv(data_dir + 'right/', convert_to_grayscale=True)

	size = (44,115)

	stitched_images = np.array([resize_images([image_processing.rectify_stitch_stereo_image(image_left, image_right, info_left, info_right)[0]], size)[0] for image_left, image_right in zip(images_left, images_right)])
	stitched_images_fov = image_processing.rectify_stitch_stereo_image(images_left[0], images_right[0], info_left, info_right)[1]
	stitched_images_fov = cv2.resize(stitched_images_fov.reshape(-1,1), (1,stitched_images[0].shape[1])).flatten()
	stitched_images_fov -= stitched_images_fov[(stitched_images_fov.size-1)/2]

	stitched_images_original = np.array([resize_images([image_processing.stitch_stereo_image(image_left, image_right)], size)[0] for image_left, image_right in zip(images_left, images_right)])

	thetas = post_processing.load_poses(data_dir).theta

	offsets = np.array([image_processing.xcorr_match_images(stitched_images[0], image)[0] for image in stitched_images])
	offsets = [stitched_images_fov[offset + (stitched_images_fov.size-1)/2] for offset in offsets]
	# offsets_2 = np.array([image_processing.xcorr_match_images(stitched_images[0], image, 10)[0] for image in stitched_images])
	offsets_original = np.array([image_processing.xcorr_match_images(stitched_images_original[0], image)[0] for image in stitched_images_original])

	n = 90
	debug_image1 = image_processing.xcorr_match_images_debug(stitched_images[0], stitched_images[n])[2]
	debug_image2 = image_processing.xcorr_match_images_debug(stitched_images_original[0], stitched_images_original[n])[2]
	debug_image = cv2.resize(np.hstack((debug_image1,debug_image2)), None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST)
	# cv2.imshow('a', 0.5+0.5*small_images[5])
	cv2.imshow('a', debug_image)
	cv2.waitKey()

	# plt.plot(thetas * math.degrees(1))
	plt.plot(offsets)
	# plt.plot(-offsets_2 * stitched_images_fov / stitched_images[0].shape[1] / 10.)
	# plt.plot(-offsets_left * 175.2 / stitched_images[0].shape[1])
	# plt.plot(-offsets_right * 175.2 / stitched_images[0].shape[1])
	plt.plot(-offsets_original * 175.2 / stitched_images[0].shape[1])

	# np.save(test_dir+'offset_data.npy', offsets_data)

import post_processing
matplotlib.style.use('ggplot')

#### Test different resolutions and subpixel correlation over a whole run
## RMS error decreases and generally looks good
ref_dir = '/home/dominic/miro/data/left-right2/'
test_full_path_offsets(ref_dir)
# test_downscaling(ref_dir, [8, 40, 80], False)
# test_depth(ref_dir, 8, False)

plt.show()