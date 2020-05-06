#!/usr/bin/python

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

def read_file(filename):
	with open(filename, 'r') as f:
		data = f.read()
	return data

dir1 = os.path.expanduser('~/miro/data/follow-long-path/')
dir2 = os.path.expanduser('~/miro/data/follow-long-path_tests/2/')
base_file_path = dir2 #+ 'full_'
# dir2 += 'norm/'

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

correlations = np.zeros((len(images1),len(images2)))
offsets = np.zeros((len(images1),len(images2)))

N = len(images1) * len(images2)
print('%d total images' % N)

start_time = time.time()

for i,image1 in enumerate(images1):
	print('progress = %.2f%% [%d/%d]'%(i*100.0/len(images1),i*len(images2), N))
	for j,image2 in enumerate(images2):
		offset, correlation = image_processing.xcorr_match_images(image1, image2)
		correlations[i,j] = correlation
		offsets[i,j] = offset

# save before normalisation
np.save(base_file_path+'c', correlations)
np.save(base_file_path+'o', offsets)

correlations /= np.reshape(np.max(correlations, axis=0), (1,-1))

dt = time.time()-start_time
print('processed %d images in %.2f seconds (%.2f / s)' % (N, dt, N/dt))

fancy = False
if fancy:
	confusion_image = np.zeros(((1+len(images1))*images1[0].shape[0],(1+len(images2))*images1[0].shape[1]), dtype=np.uint8)

	for i, image1 in enumerate(images1):
		confusion_image[(i+1)*images1[0].shape[0]:(i+2)*images1[0].shape[0],:images1[0].shape[1]] = (1 + image1) * 255.0/2.0

	for j,image2 in enumerate(images2):
		confusion_image[:images1[0].shape[0],(j+1)*images1[0].shape[1]:(j+2)*images1[0].shape[1]] = (1 + image2) * 255.0/2.0

	for i in range(len(images1)):
		for j in range(len(images2)):
			confusion_image[(i+1)*images1[0].shape[0]:(i+2)*images1[0].shape[0],(j+1)*images1[0].shape[1]:(j+2)*images1[0].shape[1]] = 255.0 * correlations[i,j]
			# num = "%.2f" % correlations[i,j]
			# textsize = cv2.getTextSize(num, cv2.FONT_HERSHEY_SIMPLEX, 1, 1)[0]
			# colour = (255,255,255)
			# if correlations[i,j] >= 0.8:
			# 	colour = (0,0,0)
			# cv2.putText(confusion_image, num, (int((j+1.5)*images1[0].shape[1]-textsize[0]/2), int((i+2)*images1[0].shape[0]-textsize[1]/2)), cv2.FONT_HERSHEY_SIMPLEX, 1, colour, 1 )
else:
	confusion_image = np.uint8(255*correlations)
	offset_image = np.uint8(255.0/115.0/2.0*abs(offsets))

# cv2.imshow('con', confusion_image)
cv2.imwrite(base_file_path+'c1.png', confusion_image)
cv2.imwrite(base_file_path+'o1.png', offset_image)

width_on_height_scale = round(confusion_image.shape[1] / float(confusion_image.shape[0]))

if width_on_height_scale > 1:
	cv2.imwrite(base_file_path+'c2.png', cv2.resize(confusion_image,None,fx=1,fy=width_on_height_scale,interpolation=cv2.INTER_NEAREST))
	cv2.imwrite(base_file_path+'o2.png', cv2.resize(offset_image,None,fx=1,fy=width_on_height_scale,interpolation=cv2.INTER_NEAREST))


cv2.waitKey()