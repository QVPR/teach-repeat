#!/usr/bin/env python

import numpy as np
import cv2
import os
import pickle
import time
import json
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

def test_image_resize(img1, img2, size_range):
	cfull, cfull_max = plot_corr(img1, img2)
	print('1 \t[%dx%d]: \tbest = %f' % (img1.shape[1], img1.shape[0], cfull_max))
	colours = [col['color'] for col in list(plt.rcParams['axes.prop_cycle'])]
	plt.axvline(cfull_max, linestyle='--')

	bests = []
	bests_sparse = []
	sizes = size_range
	for n, i in enumerate(sizes):
		resized_images = resize_images([img1, img2], 1./i, interp=cv2.INTER_AREA)
		# corr, best, best_sparse = plot_corr(*resized_images, subsample=i, color=colours[(n+1) % len(colours)])
		corr, best, best_sparse = plot_corr(*patch_normalise_images(resized_images, (9,9)), subsample=i, color=colours[(n+1) % len(colours)])
		print('%d\t[%dx%d]:\tbest = %.2f\tbest_sparse = %.2f' % (i, resized_images[0].shape[1], resized_images[0].shape[0], best, best_sparse))
		bests.append(best)
		bests_sparse.append(best_sparse)
	if len(size_range) < 10:
		plt.legend()
	plt.title('Image correlation profiles for reduced resolutions')
	plt.xlabel('NCC image offset (full res px)')
	plt.ylabel('normalised correlation')

	plt.figure()
	plt.plot([0,sizes[-1]],[cfull_max,cfull_max], "k--", sizes, bests, '.', sizes, bests_sparse, '.')
	plt.title('Subpixel correlation offsets for small image sizes')
	plt.legend(['full res image','subpixel','pixel'])
	plt.xlabel('image size reduction factor')
	plt.ylabel('NCC image offset (full res px)')

def test_image_resize_aliasing(img):
	aliased = []
	for i in range(1,500):
		img_s1 = cv2.resize(img1_full, None, fx = 1./i, fy=1./i, interpolation=cv2.INTER_AREA)
		img_s2 = cv2.resize(img1_full, None, fx = 1./(i+1), fy=1./(i+1), interpolation=cv2.INTER_AREA)
		if np.all(img_s1 == img_s2):
			aliased.append((i,i+1))
	return aliased

def test_full_path_offsets(ref_dir, test_dir, sizes_to_test):
	if os.path.exists(test_dir + 'offset_data.npy'):
		offsets_data = np.load(test_dir + 'offset_data.npy', allow_pickle=True)[()]
	else:
		offsets_data = {}
	test_keyframes = post_processing.get_image_keyframes(post_processing.load_images_cv(test_dir + 'norm/'), post_processing.load_images_cv(test_dir))
	test_images = post_processing.load_images_cv(test_dir + 'norm/')
	test_keyframe_correspondances = np.array([np.argmin([np.sum(np.abs(keyframe-test_image)) for test_image in test_images]) for keyframe in test_keyframes])
	correspondances_full = post_processing.get_full_correspondances(test_keyframe_correspondances, test_images.shape[0])

	ref_full = post_processing.get_sorted_files_by_ending(ref_dir+'full/', '.png')
	test_full = post_processing.get_sorted_files_by_ending(test_dir+'full/', '.png')

	for size in sizes_to_test:
		if str(size) in offsets_data:
			continue
		offsets_data[str(size)] = []
		if size != 1:
			offsets_data[str(size)+'_sub'] = []
		print('calculating size %d' % size)
		for i, c in enumerate(correspondances_full):
			if i % 10 == 0:
				print('%d/%d' % (i, len(correspondances_full)))
			ref_image = cv2.imread(ref_full[c], cv2.IMREAD_GRAYSCALE)
			test_image = cv2.imread(test_full[i], cv2.IMREAD_GRAYSCALE)
			if size != 1:
				ref_image, test_image = resize_images([ref_image, test_image], 1./size)
				# ref_image, test_image = patch_normalise_images(resize_images([ref_image, test_image], 1./size), (9,9))
			ref_image = np.pad(ref_image, ((0,),(int(test_image.shape[1]/2),)), mode='constant', constant_values=0)
			corr = image_processing.normxcorr2_subpixel(ref_image, test_image, size)
			offset = np.argmax(corr) - (len(corr)-1)/2
			
			if size == 1:
				offsets_data[str(size)].append(offset)
			else:
				offset_not_sub = size * np.argmax(corr[::size]) - (len(corr)-1)/2
				offsets_data[str(size)+'_sub'].append(offset)
				offsets_data[str(size)].append(offset_not_sub)

	np.save(test_dir+'offset_data.npy', offsets_data)

	plt.figure()
	for size in sizes_to_test:
		plt.plot(offsets_data[str(size)], label=str(size))
		if size != 1:
			plt.plot(offsets_data[str(size)+'_sub'], label=str(size)+'_sub')
	plt.legend()

	pixel_accuracy_rms = collections.OrderedDict()
	subpixel_accuracy_rms = collections.OrderedDict()
	for size in sizes_to_test:
		if size != 1:
			not_sub_rms = math.sqrt(((np.array(offsets_data[str(size)]) - np.array(offsets_data['1']))**2).mean())
			sub_rms = math.sqrt(((np.array(offsets_data[str(size)+'_sub']) - np.array(offsets_data['1']))**2).mean())
			print('RMS [%d]:\tdiscrete: %.3f\tsubpixel: %.3f' % (size, not_sub_rms, sub_rms))
			im_size = '[%dx%d]' % (round(925./size), round(360./size))
			pixel_accuracy_rms[im_size] = not_sub_rms
			subpixel_accuracy_rms[im_size] = sub_rms
			# plt.figure()
			mod_offset = 1. / size * ((np.array(offsets_data['1'])) % size)
			mod_offset_05 = 1. / size * ((np.array(offsets_data['1']) + size/2.) % size) - 0.5
			sub_err = np.float64(offsets_data[str(size)+'_sub']) - np.array(offsets_data['1'])
			px_err = np.float64(offsets_data[str(size)]) - np.array(offsets_data['1'])
			bad_err = abs(sub_err / px_err)
			bad_err[px_err == 0] = 0
			# plt.plot(mod_offset)
			# plt.scatter(mod_offset_05, sub_err, alpha=0.2)
			# plt.scatter(mod_offset_05, px_err)
			# plt.plot(bad_err, '.')
			# plt.plot(offsets_data[str(size)] - np.array(offsets_data['1']))
			n = np.argmax(bad_err)
			print(n)
			bad_ref = np.float64(cv2.imread(ref_full[correspondances_full[n]], cv2.IMREAD_GRAYSCALE))
			bad_test = np.float64(cv2.imread(test_full[n], cv2.IMREAD_GRAYSCALE))
			off, corr, img = image_processing.xcorr_match_images_debug(bad_ref, bad_test, 1)
			bad_ref, bad_test = patch_normalise_images(resize_images([bad_ref, bad_test], 1./size), (9,9))
			off2, corr2, img2 = image_processing.xcorr_match_images_debug(bad_ref, bad_test, 1)
			print(off, off2)
			print(offsets_data[str(size)+'_sub'][n], offsets_data['1'][n])
			# cv2.imshow('full',cv2.resize(img,None,fx=1.0,fy=0.5,interpolation=cv2.INTER_NEAREST))
			cv2.imshow('sub',cv2.resize(img2,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
			cv2.waitKey()



	combined = collections.OrderedDict((('pixel accuracy', pixel_accuracy_rms), ('subpixel accuracy', subpixel_accuracy_rms)))
	df = pd.DataFrame(combined)
	df = df.iloc[np.argsort([int(s[1:s.index('x')]) for s in list(df.index)])[::-1]]
	df.plot(kind='bar')
	plt.xticks(rotation=0)
	plt.title('Correlation error vs full-res image')
	plt.ylabel('RMS error (px)')

def test_execution_time(img1, img2, sizes):
	results = []
	stds = []
	for size in sizes:
		img1_ = img1
		img2_ = img2
		if size != 1:
			img1_, img2_ = resize_images([img1_, img2_], 1./size)
		img1_ = np.pad(img1_, ((0,),(int(img2_.shape[1]/2),)), mode='constant', constant_values=0)
		t = []
		for i in range(10):
			t1 = time.time()
			corr = image_processing.normxcorr2_subpixel(img1_, img2_, size)
			# corr = image_processing.normxcorr2_subpixel_fast(img1_, img2_, size)
			t2 = time.time()
			t.append(t2-t1)
		t = np.array(t)
		results.append(t.mean())
		stds.append(t.std())
	df = pd.DataFrame(results, index=sizes)
	df.plot(kind='bar', yerr = stds, label='execution time')

import post_processing
matplotlib.style.use('ggplot')

img1_full = cv2.imread('/home/dominic/miro/data/under-table2/full/000046.png', cv2.IMREAD_GRAYSCALE)
img2_full = cv2.imread('/home/dominic/miro/data/under-table2_tests/3/full/000802.png', cv2.IMREAD_GRAYSCALE)
# img1_full = cv2.imread('/home/dominic/miro/data/follow-long-path/full/000003.png', cv2.IMREAD_GRAYSCALE)
# img2_full = cv2.imread('/home/dominic/miro/data/follow-long-path_tests/1/full/000061.png', cv2.IMREAD_GRAYSCALE)
img1_depth = np.load('/home/dominic/miro/data/under-table2/full/000046_disp.npy').squeeze()
img1_depth = cv2.resize(img1_depth, img1_full.shape[::-1])

# img1_full, img1_depth, img2_full = resize_images([img1_full, img1_depth, img2_full], (44,115))
# img1_pn = patch_normalise_images([img1_full])[0]

a = np.clip(1.0 - 1.0*(np.abs(np.arange(img1_full.shape[1]) - (img1_full.shape[1]/2.)).reshape((1,-1)) / (img1_full.shape[1]/2.))**2, 0, 1)
b = np.clip(0.2 + np.linspace(0,1,img1_full.shape[0]).reshape(-1,1)**0.5, 0, 1)
a = b * a

cv2.imshow('a',a)
cv2.imshow('depth',img1_depth)
cv2.imshow('both',a*img1_depth)
cv2.imshow('img1', np.uint8(a*img1_depth*img1_full))
# cv2.imshow('img1_depth', np.uint8(256/2*a*img1_depth*(1+img1_pn)))
cv2.waitKey()

_, best1 = plot_corr(img1_full, img2_full)
# plot_corr((0.25+0.75*a*img1_depth)*img1_full, img2_full)``
_, best2 = plot_corr(a*img1_depth*img1_full, img2_full)
print(best1, best2)
# plt.show()

im1 = 0.5*img1_full + 0.5*image_processing.subpixel_shift_approx(img2_full, best1)
im2 = 0.5*img1_full + 0.5*image_processing.subpixel_shift_approx(img2_full, best2)
cv2.imshow('1', np.uint8(im1))
cv2.imshow('2', np.uint8(im2))
cv2.waitKey()

#### Test image aliasing:
## Is there a point where the image is the same when resized by 1/n and 1/(n+1)
## Yes - seems to be a constant function of initial image size and interpolation approach
## But happens first when the resulting image in 3x1 or smaller
# print(test_image_resize_aliasing(img1_full))
# print(test_image_resize_aliasing(img2_full))

#### Test effect of subsampling after resizing the images:
## correlation profiles look good - subpixel seems to help
# test_image_resize(img1_full, img2_full, [8, 20, 40, 80, 160])
## isn't perfect but generally performs a lot better down to lower resolutions
# test_image_resize(img1_full, img2_full, range(2,163,3))

#### Test the execution time of the subpixel correlation
## after 1/20th size [46x18] the overhead of subsampling seems to make things slower - trade off
# test_execution_time(img1_full, img2_full, [1, 8, 20, 40, 80, 160])

#### Test different resolutions and subpixel correlation over a whole run
## RMS error decreases and generally looks good
# ref_dir = '/home/dominic/miro/data/under-table2/'
# test_dir = '/home/dominic/miro/data/under-table2_tests/3/'
# test_full_path_offsets(ref_dir, test_dir, [1, 8])

# plt.show()