#!/usr/bin/env python

import numpy as np
import cv2
import os
import pickle
import time
import json
import math
import sys

sys.path.append(os.path.dirname(__file__) + '/../nodes')
import importlib
image_processing = importlib.import_module('image_processing')

import colourmap

def read_file(filename):
	with open(filename, 'r') as f:
		data = f.read()
	return data

dir1 = os.path.expanduser('~/miro/data/follow-long-path/')
dir2 = os.path.expanduser('~/miro/data/follow-long-path_tests/17/')
dir2_full = os.path.expanduser('~/miro/data/follow-long-path_tests/17/norm/')
base_file_path = dir2
# dir2 += 'norm/'

half_search_range = 5

image_files1 = [dir1+f for f in os.listdir(dir1) if f[-10:] == '_image.pkl']
image_files1.sort()
images1 = [pickle.loads(read_file(image)) for image in image_files1]

if dir2[-5:-1] == 'norm':
	image_files2 = [dir2+f for f in os.listdir(dir2) if f[-4:] == '.png']
	image_files2.sort()
	images2 = [np.float64(cv2.imread(image, cv2.IMREAD_GRAYSCALE))*2.0/255.0 - 1.0 for image in image_files2]
else:
	image_files2 = [dir2+f for f in os.listdir(dir2) if f[-10:] == '_image.pkl']
	image_files2.sort()
	images2 = [pickle.loads(read_file(image)) for image in image_files2]
if dir2_full[-5:-1] == 'norm':
	image_files2_full = [dir2_full+f for f in os.listdir(dir2_full) if f[-4:] == '.png']
	image_files2_full.sort()
	images2_full = [np.float64(cv2.imread(image, cv2.IMREAD_GRAYSCALE))*2.0/255.0 - 1.0 for image in image_files2_full]
else:
	image_files2_full = [dir2_full+f for f in os.listdir(dir2_full) if f[-10:] == '_image.pkl']
	image_files2_full.sort()
	images2_full = [pickle.loads(read_file(image)) for image in image_files2_full]

if not os.path.exists(base_file_path+'confusion.npy'):

	correlations = np.zeros((len(images1),len(images2_full)))

	N = len(images1) * len(images2_full)
	print('%d total images' % N)

	start_time = time.time()

	# # for i,image1 in enumerate(images1):
	# for j,image2 in enumerate(images2_full):
	# 	if j < correspondances[0] or j >= correspondances[-1]:
	# 		continue
	# 	current_index = np.where(correspondances > j)[0][0] - 1
	# 	print('progress = %.2f%%'%(j*100.0/len(images2_full)))
	# 	# for j,image2 in enumerate(images2):
	# 	for i in range(max(0,current_index-half_search_range), min(len(images1),current_index+half_search_range+1)):
	# 		image1 = images1[i]
	# 		offset, correlation = image_processing.xcorr_match_images(image1, image2)
	# 		# correlation = image_processing.normxcorr2(image1, image2)
	# 		correlations[i,j] = correlation

	for i,image1 in enumerate(images1):
		print('progress = %.2f%% [%d/%d]'%(i*100.0/len(images1),i*len(images2_full), N))
		for j,image2 in enumerate(images2_full):
			offset, correlation = image_processing.xcorr_match_images(image1, image2)
			correlations[i,j] = correlation

	# save before normalisation
	np.save(base_file_path+'c', correlations)

	confusion_image_unscaled = np.uint8(255*colourmap.parula_data[np.uint8(255*correlations)])[:,:,::-1]

	confusion_image_global_scaled = np.uint8(255*colourmap.parula_data[np.uint8(255*(correlations/correlations.max()))])[:,:,::-1]

	correlations /= np.reshape(np.max(correlations, axis=0), (1,-1))

	dt = time.time()-start_time
	print('processed %d images in %.2f seconds (%.2f / s)' % (N, dt, N/dt))

	confusion_image = np.uint8(255*colourmap.parula_data[np.uint8(255*correlations)])[:,:,::-1]

	cv2.imwrite(base_file_path+'c1.png', confusion_image)
	cv2.imwrite(base_file_path+'c1_unscaled.png', confusion_image_unscaled)
	cv2.imwrite(base_file_path+'c1_global_scaled.png', confusion_image_global_scaled)

	width_on_height_scale = round(confusion_image.shape[1] / float(confusion_image.shape[0]))

	if width_on_height_scale > 1:
		cv2.imwrite(base_file_path+'c2.png', cv2.resize(confusion_image,None,fx=1,fy=width_on_height_scale,interpolation=cv2.INTER_NEAREST))
		cv2.imwrite(base_file_path+'c2_unscaled.png', cv2.resize(confusion_image_unscaled,None,fx=1,fy=width_on_height_scale,interpolation=cv2.INTER_NEAREST))
		cv2.imwrite(base_file_path+'c2_global_scaled.png', cv2.resize(confusion_image_global_scaled,None,fx=1,fy=width_on_height_scale,interpolation=cv2.INTER_NEAREST))
else:
	correlations = np.load(base_file_path+'confusion.npy')

x = np.array([int(image_name[-10:-4]) for image_name in image_files2_full])

correspondances_indices = np.zeros(len(images2), dtype=np.int)
for i,image1 in enumerate(images2):
	diffs = [np.sum(np.abs(image1-images2_full[j])) for j in range(len(images2_full))]
	correspondances_indices[i] = np.argmin(diffs)
correspondances = x[correspondances_indices]

import matplotlib
import matplotlib.pyplot as plt

correlations = correlations[:,correspondances_indices[0]:correspondances_indices[-1]]
x = x[correspondances_indices[0]:correspondances_indices[-1]]

step_size = float(correlations.shape[0]) / correlations.shape[1]

best_matches = np.argmax(correlations, axis=0)
combo = np.zeros_like(best_matches, dtype=np.float)
combo[0] = -1
for i in range(1,len(combo)):
	# combo[i] = combo[i-1] + step_size + 0.02*(best_matches[i]-combo[i-1])
	combo[i] = combo[i-1] + 1.0*int(np.any(correspondances_indices == i-1+correspondances_indices[0])) + 0.01*(best_matches[i]-combo[i-1])

matplotlib.style.use('ggplot')
plt.plot(x/15.0, best_matches*0.2, '.', alpha = 0.4)
plt.plot(correspondances/15.0, np.arange(len(correspondances))*0.2, 'x')
plt.title('Along-route localisation vs best visual match [110% odom]')
plt.xlabel('time (s)')
plt.ylabel('path distance (m)')
plt.legend(['best visual match','keyframe'])

plt.show()