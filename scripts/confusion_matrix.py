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

def read_file(filename):
	with open(filename, 'r') as f:
		data = f.read()
	return data

def confusion_matrix(images1, images2):
	N = len(images1) * len(images2)

	correlations = np.zeros((len(images1),len(images2)))
	offsets = np.zeros((len(images1),len(images2)))

	for i,image1 in enumerate(images1):
		print('progress = %.2f%% [%d/%d]'%(i*100.0/len(images1),i*len(images2), N))
		for j,image2 in enumerate(images2):
			offset, correlation = image_processing.xcorr_match_images(image1, image2, 2)
			correlations[i,j] = correlation
			offsets[i,j] = offset / 2.

	return correlations, offsets

def confusion_image(matrix, fancy=False):
	if fancy:
		image = np.zeros(((1+len(images1))*images1[0].shape[0],(1+len(images2))*images1[0].shape[1]), dtype=np.uint8)

		for i, image1 in enumerate(images1):
			image[(i+1)*images1[0].shape[0]:(i+2)*images1[0].shape[0],:images1[0].shape[1]] = (1 + image1) * 255.0/2.0

		for j,image2 in enumerate(images2):
			image[:images1[0].shape[0],(j+1)*images1[0].shape[1]:(j+2)*images1[0].shape[1]] = (1 + image2) * 255.0/2.0

		for i in range(len(images1)):
			for j in range(len(images2)):
				image[(i+1)*images1[0].shape[0]:(i+2)*images1[0].shape[0],(j+1)*images1[0].shape[1]:(j+2)*images1[0].shape[1]] = 255.0 * correlations[i,j]
				num = "%.2f" % matrix[i,j]
				textsize = cv2.getTextSize(num, cv2.FONT_HERSHEY_SIMPLEX, 1, 1)[0]
				colour = (255,255,255)
				if matrix[i,j] >= 0.8:
					colour = (0,0,0)
				cv2.putText(image, num, (int((j+1.5)*images1[0].shape[1]-textsize[0]/2), int((i+2)*images1[0].shape[0]-textsize[1]/2)), cv2.FONT_HERSHEY_SIMPLEX, 1, colour, 1 )
	else:
		image = np.uint8(255*matrix.clip(min=0))
	return image


if __name__ == "__main__":
	dir1 = os.path.expanduser('~/miro/data/follow-long-path/')
	dir2 = os.path.expanduser('~/miro/data/follow-long-path_tests/2/norm/')
	base_file_path = os.path.expanduser('~/miro/data/follow-long-path_tests/2/')

	image_files1 = [dir1+f for f in os.listdir(dir1) if f[-10:] == '_image.pkl']
	image_files1.sort()
	images1 = [pickle.loads(read_file(image)) for image in image_files1]

	image_files2 = [dir2+f for f in os.listdir(dir2) if f[-4:] == '.png']
	image_files2.sort()
	images2 = [np.float64(cv2.imread(image, cv2.IMREAD_GRAYSCALE))*2.0/255.0 - 1.0 for image in image_files2]

	correlations, offsets = confusion_matrix(images1, images2)

	# save before normalisation
	np.save(base_file_path+'confusion', correlations)
	np.save(base_file_path+'offsets', offsets)

	correlations /= np.reshape(np.max(correlations, axis=0), (1,-1))

	confusion_image = confusion_image(correlations)
	offset_image = confusion_image(offsets)

	cv2.imwrite(base_file_path+'confusion1.png', confusion_image)
	cv2.imwrite(base_file_path+'offsets1.png', offset_image)

	width_on_height_scale = round(confusion_image.shape[1] / float(confusion_image.shape[0]))

	if width_on_height_scale > 1:
		cv2.imwrite(base_file_path+'confusion2.png', cv2.resize(confusion_image,None,fx=1,fy=width_on_height_scale,interpolation=cv2.INTER_NEAREST))
		cv2.imwrite(base_file_path+'offsets2.png', cv2.resize(offset_image,None,fx=1,fy=width_on_height_scale,interpolation=cv2.INTER_NEAREST))