#!/usr/bin/python

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

import importlib
sys.path.append(os.path.dirname(__file__) + '/../nodes')
image_processing = importlib.import_module('image_processing')

FULL_RES = 925.

def corr_images(img1, img2, subsample=1):
	img1_pad = np.pad(img1, ((0,),(int(img2.shape[1]/2),)), mode='constant', constant_values=0)
	return image_processing.normxcorr2_subpixel(img1_pad, img2, subsample, 'valid')

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

	bests = []
	bests_sparse = []
	sizes = size_range
	for n, i in enumerate(sizes):
		resized_images = resize_images([img1, img2], 1./i)
		corr, best, best_sparse = plot_corr(*resized_images, subsample=i, color=colours[(n+1) % len(colours)])
		# corr, best, best_sparse = plot_corr(*patch_normalise_images(resized_images, (9,9)), subsample=i, color=colours[(n+1) % len(colours)])
		print('%d \t[%dx%d]: \tbest = %f, \tbest_sparse = %f' % (i, resized_images[0].shape[1], resized_images[0].shape[0], best, best_sparse))
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

import post_processing
matplotlib.style.use('ggplot')

img1_full = cv2.imread('/home/dominic/miro/data/under-table2/full/000046.png', cv2.IMREAD_GRAYSCALE)
img2_full = cv2.imread('/home/dominic/miro/data/under-table2_tests/3/full/000802.png', cv2.IMREAD_GRAYSCALE)

#### Test image aliasing:
## Is there a point where the image is the same when resized by 1/n and 1/(n+1)
## Yes - seems to be a constant function of initial image size and interpolation approach
## But happens first when the resulting image in 3x1 or smaller
# print(test_image_resize_aliasing(img1_full))
# print(test_image_resize_aliasing(img2_full))

#### Test effect of subsampling after resizing the images:
## correlation profiles look good - subpixel seems to help
# test_image_resize(img1_full, img2_full, [8, 20, 40, 80])
## isn't perfect but generally performs a lot better down to lower resolutions
test_image_resize(img1_full, img2_full, range(2,163,3))



plt.show()