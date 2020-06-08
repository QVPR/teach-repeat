#!/usr/bin/python

import cv_bridge
import cv2
import math
import numpy as np
import numpy.lib
import scipy.signal

bridge = cv_bridge.CvBridge()

def patch_normalise(image, patch_size):
	patch_half_size = [(p-1)/2 for p in patch_size]
	height, width = image.shape
	out = np.zeros(image.shape, dtype=float)
	with np.errstate(divide='ignore', invalid='ignore'):
		for y in range(height):
			for x in range(width):
				patch = image[	max(0, y-patch_half_size[1]):min(height, y+1+patch_half_size[1]),
								max(0, x-patch_half_size[0]):min(width, x+1+patch_half_size[0])]

				mu, stdev = mean_stdev_fast(patch)
				out[y,x] = (image[y,x] - mu) / stdev

	out[np.isnan(out)] = 0.0
	out[out < -1.0] = -1.0
	out[out > 1.0] = 1.0
	return out

def patch_normalise_patch(image, patch_size):
	patch_half_size = [(p-1)/2 for p in patch_size]
	height, width = image.shape
	out = np.zeros(image.shape, dtype=float)

	nrows = image.shape[0] - patch_size[0] + 1
	ncols = image.shape[1] - patch_size[1] + 1
	patches = get_patches2D(image, patch_size)
	mus, stds = mean_stdev_fast(patches, 0)

	with np.errstate(divide='ignore', invalid='ignore'):
		for y in range(height):
			for x in range(width):
				if x < patch_half_size[1] or y < patch_half_size[0] or width-x <= patch_half_size[1] or height-y <= patch_half_size[0]:
					patch = image[	max(0, y-patch_half_size[1]):min(height, y+1+patch_half_size[1]),
									max(0, x-patch_half_size[0]):min(width, x+1+patch_half_size[0])]
					mu, stdev = mean_stdev_fast(patch)
					out[y,x] = (image[y,x] - mu) / stdev

		yr = (patch_half_size[1], patch_half_size[1]+nrows)
		xr = (patch_half_size[0], patch_half_size[0]+ncols)
		out[yr[0]:yr[1], xr[0]:xr[1]] = (image[yr[0]:yr[1], xr[0]:xr[1]] - mus.reshape(nrows, ncols)) / stds.reshape(nrows, ncols)

	out[np.isnan(out)] = 0.0
	out[out < -1.0] = -1.0
	out[out > 1.0] = 1.0
	return out

def patch_normalise_pad(image, patch_size):
	patch_half_size = [(p-1)/2 for p in patch_size]
	height, width = image.shape

	image_pad = np.pad(np.float64(image), patch_half_size, 'constant', constant_values=np.nan)

	nrows = image.shape[0]
	ncols = image.shape[1]
	patches = get_patches2D(image_pad, patch_size)
	mus = np.nanmean(patches, 0)
	stds = np.nanstd(patches, 0)

	with np.errstate(divide='ignore', invalid='ignore'):
		out = (image - mus.reshape(nrows, ncols)) / stds.reshape(nrows, ncols)

	out[np.isnan(out)] = 0.0
	out[out < -1.0] = -1.0
	out[out > 1.0] = 1.0
	return out

def grayscale(image):
	return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

def patch_normalise_msg(msg, patch_size, compressed=False, resize=None):
	# resize should be height * width
	if compressed:
		image = compressed_msg_to_image(msg)
	else:
		image = msg_to_image(msg)
	if len(image.shape) > 2 and image.shape[2] > 1:
		image = grayscale(image)
	if resize is not None:
		# opencv sizes are the opposite of numpy (width*height)
		resize = tuple(reversed(resize))
		image = cv2.resize(image, resize, interpolation=cv2.INTER_AREA)
	return patch_normalise_pad(image, patch_size)

def patch_normalise_image(image, patch_size, resize=None):
	# resize should be height * width
	if len(image.shape) > 2 and image.shape[2] > 1:
		image = grayscale(image)
	if resize is not None:
		# opencv sizes are the opposite of numpy (width*height)
		resize = tuple(reversed(resize))
		image = cv2.resize(image, resize, interpolation=cv2.INTER_AREA)
	return patch_normalise_pad(image, patch_size)

def make_size(height, width):
	return (height, width)

def msg_to_image(msg):
	try:
		image = bridge.imgmsg_to_cv2(msg)
	except cv_bridge.CvBridgeError as e:
		print(e)
		return
	return image

def compressed_msg_to_image(msg):
	try:
		image = bridge.compressed_imgmsg_to_cv2(msg)
	except cv_bridge.CvBridgeError as e:
		print(e)
		return 
	return image

def image_to_msg(image, encoding='passthrough'):
	try:
		msg = bridge.cv2_to_imgmsg(image, encoding=encoding)
	except cv_bridge.CvBridgeError as e:
		print(e)
		return
	return msg

def get_patches2D(image, patch_size):
	nrows = image.shape[0] - patch_size[0] + 1
	ncols = image.shape[1] - patch_size[1] + 1
	return numpy.lib.stride_tricks.as_strided(image, patch_size + (nrows, ncols), image.strides + image.strides).reshape(patch_size[0]*patch_size[1],-1)

def get_patches2D_sparse(image, patch_size, point_stride, patch_type='all'):
	full_patch_size_rows = 1 + (patch_size[0]-1) * point_stride[0]
	full_patch_size_cols = 1 + (patch_size[1]-1) * point_stride[1]
	nrows = image.shape[0] - full_patch_size_rows + 1
	ncols = image.shape[1] - full_patch_size_cols + 1
	if patch_type == 'all':
		output_size = (patch_size[0], patch_size[1], nrows, ncols)
	elif patch_type == 'rows':
		output_size = (patch_size[0], patch_size[1], nrows, 1)
	elif patch_type == 'cols':
		output_size = (patch_size[0], patch_size[1], 1, ncols)
	elif patch_type == 'one':
		output_size = (patch_size[0], patch_size[1], 1, 1)
	strides = (point_stride[0]*image.strides[0], point_stride[1]*image.strides[1]) + image.strides
	return numpy.lib.stride_tricks.as_strided(image, output_size, strides).reshape(patch_size[0]*patch_size[1],-1)

def get_patches1D(image, patch_size):
	nrows = image.shape[0] - patch_size + 1
	return numpy.lib.stride_tricks.as_strided(image, (patch_size, nrows), image.strides + image.strides)

def mean_stdev_fast(array, axis=None):
	mu = np.mean(array, axis)
	sigma = np.sqrt(((array - mu)**2).mean(axis))
	return mu, sigma

def horizontal_SAD_match_images(image, template_image, template_proportion=0.5, vertical_cutoff=0.66):
	# horizontal proportion of template
	template_width = int(template_image.shape[1]*template_proportion)
	template_start = (template_image.shape[1]-template_width) // 2
	template = template_image[:,template_start:template_start+template_width]

	# vertical proportion of image and template
	image = image[:int(image.shape[0]*vertical_cutoff)]
	template = template[:int(template.shape[0]*vertical_cutoff)]

	(offset, error) = scan_horizontal_SAD_match(image, template)
	return offset-template_start, error

def xcorr_match_images(image, template_image):
	image = np.pad(image, ((0,),(int(template_image.shape[1]/2),)), mode='constant', constant_values=0)
	corr = normxcorr2(image, template_image, mode='valid')
	offset = np.argmax(corr)
	return offset - int(template_image.shape[1]/2), corr[0,offset]

def xcorr_match_images_debug(image, template_image):
	image_pad = np.pad(image, ((0,),(int(template_image.shape[1]/2),)), mode='constant', constant_values=0)
	corr = normxcorr2(image_pad, template_image, mode='valid')
	offset = np.argmax(corr)
	debug_image = create_correlation_debug_image(image, template_image, corr)
	return offset - int(template_image.shape[1]/2), corr[0,offset], debug_image

def scan_horizontal_SAD_match(image, template, step_size=1):
	positions = range(0,image.shape[1]-template.shape[1],step_size)
	differences = [0] * len(positions)
	for i,pos in enumerate(positions):
		differences[i] = np.mean(np.abs(image[:,pos:pos+template.shape[1]] - template))
	index = np.argmin(differences)
	return positions[index], differences[index]

def scan_horizontal_SAD_match_pad(image, template, step_size=1):
	image_pad = np.pad(image, ((0,),(template.shape[1]-1,)), 'constant', constant_values=np.nan)
	positions = range(0,image.shape[1],step_size)
	differences = [0] * len(positions)
	for i,pos in enumerate(positions):
		differences[i] = np.nanmean(np.abs(image_pad[:,pos:pos+template.shape[1]] - template))
	index = np.argmin(differences)
	return positions[index]-(template.shape[1]-1), differences[index]

def scan_horizontal_SAD_match_patches(image, template):
	patches = get_patches2D(image, template.shape)
	differences = np.mean(np.abs(patches - template.ravel().reshape(-1,1)), 0)
	index = np.argmin(differences)
	return index, differences[index]

def stitch_stereo_image(image_left, image_right):
	overlap_proportion = 67.2 / 121.2
	overlap_pixels = int(round(image_left.shape[1] * overlap_proportion))
	non_overlap_pixels = image_left.shape[1] - overlap_pixels
	full_width = image_left.shape[1] + image_right.shape[1] - overlap_pixels

	if len(image_left.shape) > 2 and image_left.shape[2] > 1:
		image_left = grayscale(image_left)
		image_right = grayscale(image_right)

	stitched_image = np.zeros((image_left.shape[0],full_width))
	blend_map_linear = np.concatenate((np.ones(non_overlap_pixels),np.arange(1,0,-1.0/(overlap_pixels+1))[1:]))
	stitched_image[:,:image_left.shape[1]] += image_left * blend_map_linear
	stitched_image[:,-image_right.shape[1]:] += image_right * np.flip(blend_map_linear)

	return stitched_image

def stitch_stereo_image_message(msg_left, msg_right, compressed=False):
	if compressed:
		return stitch_stereo_image(compressed_msg_to_image(msg_left),compressed_msg_to_image(msg_right))
	else:
		return stitch_stereo_image(msg_to_image(msg_left),msg_to_image(msg_right))

def normxcorr2(image, template, mode="full"):
	image = image.copy()
	template = template.copy()
	image_min = image.min()
	if image_min < 0:
		image -= image_min
	template_min = template.min()
	if template_min < 0:
		template -= template_min

	xcorr = scipy.signal.correlate2d(image, template, mode=mode)

	m,n = template.shape
	mn = m*n

	if mode == "full":
		local_sum_image = local_sum_full(image,m,n)
		local_sum_image2 = local_sum_full(image**2,m,n)
	elif mode == "same":
		local_sum_image = local_sum_same(image,m,n)
		local_sum_image2 = local_sum_same(image**2,m,n)
	elif mode == "valid":
		local_sum_image = local_sum_valid(image,m,n)
		local_sum_image2 = local_sum_valid(image**2,m,n)
	else:
		raise NotImplementedError('normxcorr2: mode should be one of "full", "valid" or "same".')

	denominator_image = local_sum_image2 - local_sum_image**2/mn
	denominator_image[denominator_image < 0.0] = 0.0
	denominator_template = ((template - template.mean())**2).sum()
	denominator = np.sqrt(denominator_image * denominator_template)
	numerator = xcorr - local_sum_image * template.sum()/mn

	correlated = np.zeros_like(numerator)
	tolerance = math.sqrt(np.spacing(abs(denominator).max()))
	nonzero_index = np.where(denominator > tolerance)
	correlated[nonzero_index] = numerator[nonzero_index] / denominator[nonzero_index]
	return correlated

def normxcorr2_sparse(image, template, sampling=(1,1)):
	image = image.copy()
	template = template.copy()
	image_min = image.min()
	if image_min < 0:
		image -= image_min
	template_min = template.min()
	if template_min < 0:
		template -= template_min

	nrows = 1 + (template.shape[0] - 1) / sampling[0]
	ncols = 1 + (template.shape[1] - 1) / sampling[1]

	template_sparse = get_patches2D_sparse(template, (nrows, ncols), sampling, 'one')
	image_patches_sparse = get_patches2D_sparse(image, (nrows, ncols), sampling, 'cols')

	local_sum = np.sum(image_patches_sparse, axis=0)
	local_sum2 = np.sum(image_patches_sparse**2, axis=0)

	xcorr = np.sum(image_patches_sparse * template_sparse, axis=0)

	mn = template_sparse.size
	template_mean = template_sparse.sum()/mn
	numerator = xcorr - local_sum * template_mean

	denominator_image = local_sum2 - local_sum**2/mn
	denominator_image[denominator_image < 0.0] = 0.0
	denominator_template = ((template_sparse - template_mean)**2).sum()
	denominator = np.sqrt(denominator_image * denominator_template)

	correlated = np.zeros_like(numerator)
	tolerance = math.sqrt(np.spacing(abs(denominator).max()))
	nonzero_index = np.where(denominator > tolerance)
	correlated[nonzero_index] = numerator[nonzero_index] / denominator[nonzero_index]
	return correlated

def local_sum_full(A, m, n):
	B = np.pad(A, ((m,),(n,)), 'constant', constant_values=0)
	s = np.cumsum(B, axis=0)
	c = s[m:-1,:] - s[:-m-1,:]
	s = np.cumsum(c, axis=1)
	return s[:,n:-1] - s[:,:-n-1]

def local_sum_same(A, m, n):
	B = np.pad(A, ((m-1,1),(n-1,1)), 'constant', constant_values=0)
	s = np.cumsum(B, axis=0)
	c = s[m:,:] - s[:-m,:]
	s = np.cumsum(c, axis=1)
	return s[:,n:] - s[:,:-n]
	
def local_sum_valid(A, m, n):
	B = np.pad(A, ((1,0),(1,0)), 'constant', constant_values=0)
	s = B.cumsum(axis=0)
	c = s[m:,:] - s[:-m,:]
	s = c.cumsum(axis=1)
	return s[:,n:] - s[:,:-n]

def image_scanline_rotation(image1, image2, min_overlap):
	scanline1 = image1.mean(axis=0)
	scanline1 /= scanline1.sum()
	scanline2 = image2.mean(axis=0)
	scanline2 /= scanline2.sum()

	pad = image2.shape[1]-min_overlap

	scanline1 = np.pad(scanline1, (pad,), 'constant', constant_values=np.nan)
	patches = get_patches1D(scanline1, scanline2.shape[0])
	diff = abs(patches - scanline2.reshape(-1,1))
	means = np.nanmean(diff, axis=0)
	offset = np.argmin(means)
	return offset - pad, means[offset]

def image_patch_rotation(image1, image2, min_overlap):
	image1 = np.float64(image1)
	image1 /= image1.sum()
	image2 = np.float64(image2)
	image2 /= image2.sum()

	pad = image2.shape[1]-min_overlap

	image1 = np.pad(image1, ((0,),(pad,)), 'constant', constant_values=np.nan)
	patches = get_patches2D(image1, image2.shape)
	diff = abs(patches - image2.reshape((-1,1)))
	means = np.nanmean(diff, axis=0)
	offset = np.argmin(means)
	return offset - pad, means[offset]

def create_correlation_debug_image(img1, img2, corr):
	offset = np.argmax(corr) - int(img2.shape[1]/2)
	debug_size = 50

	corr_positions = np.flip(np.int32(-(debug_size-1)*np.clip(corr[0,:],0,1))) - 1

	debug_image = np.concatenate((img2, img1, -1*np.ones((debug_size,img1.shape[1]))), axis=0)
	debug_image = np.uint8(255.0 * (1 + debug_image) / 2.0)
	debug_image = cv2.merge((debug_image, debug_image, debug_image))

	cv2.line(debug_image, (int(-offset+img1.shape[1]/2),0), (int(-offset+img1.shape[1]/2),img1.shape[0]-1), (0,255,0))
	cv2.line(debug_image, (int(img1.shape[1]/2),img1.shape[0]), (int(img1.shape[1]/2), 2*img1.shape[0]-1), (255,0,0))
	
	cv2.line(debug_image, (0,debug_image.shape[0]-(debug_size/4)), (img1.shape[1],debug_image.shape[0]-(debug_size/4)), (60,0,0))
	cv2.line(debug_image, (0,debug_image.shape[0]-(debug_size/2)), (img1.shape[1],debug_image.shape[0]-(debug_size/2)), (60,0,0))
	cv2.line(debug_image, (0,debug_image.shape[0]-(3*debug_size/4)), (img1.shape[1],debug_image.shape[0]-(3*debug_size/4)), (60,0,0))
	cv2.line(debug_image, (int(img1.shape[1]/2),2*img1.shape[0]), (int(img1.shape[1]/2),debug_image.shape[0]), (60,0,0))
	
	debug_image[corr_positions,np.arange(corr_positions.size),:] = 255
	return debug_image


if __name__ == "__main__":
	np.random.seed(0)
	import pickle
	def read_file(filename):
		with open(filename, 'r') as f:
			data = f.read()
		return data

	import time

	# img1 = pickle.loads(read_file('/home/dominic/miro/data/follow-straight-odom/000000_image.pkl'))
	# img2 = pickle.loads(read_file('/home/dominic/miro/data/follow-straight-odom_tests/2 - HUGE VISION FAIL/00000_image.pkl'))
	
	img1 = pickle.loads(read_file('/home/dominic/miro/data/follow-straight-odom/000000_image.pkl'))
	img2 = pickle.loads(read_file('/home/dominic/miro/data/follow-straight-odom_tests/8/000000_image.pkl'))

	img1_pad = np.pad(img1, ((0,),(int(img2.shape[1]/2),)), mode='constant', constant_values=0)

	t1 = time.time()
	corr = normxcorr2(img1_pad, img2, mode='valid')
	t = time.time() - t1
	print('full normxcorr time: %f s' % (t))

	t1 = time.time()
	corr_1 = normxcorr2_sparse(img1_pad, img2, (1,1))
	t_1 = time.time() - t1
	print('normxcorr 1x1 time: %f s' % (t_1))

	t1 = time.time()
	corr_2 = normxcorr2_sparse(img1_pad, img2, (2,2))
	t_2 = time.time() - t1
	print('normxcorr 2x2 time: %f s' % (t_2))

	t1 = time.time()
	corr_3 = normxcorr2_sparse(img1_pad, img2, (3,3))
	t_3 = time.time() - t1
	print('normxcorr 3x3 time: %f s' % (t_3))

	t1 = time.time()
	corr_4 = normxcorr2_sparse(img1_pad, img2, (4,4))
	t_4 = time.time() - t1
	print('normxcorr 4x4 time: %f s' % (t_4))

	# debug_image = create_correlation_debug_image(img1, img2, corr)
	# cv2.imshow('img', cv2.resize(debug_image, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST))
	# cv2.waitKey()
	
	corrs = [corr_1, corr_2, corr_3, corr_4]
	times = [t_1, t_2, t_3, t_4]
	import matplotlib.pyplot as plt

	for i, (corr, t) in enumerate(zip(corrs,times)):
		plt.subplot(len(corrs)*100 + 10 + i+1)
		plt.plot(corr)
		m = np.argmax(corr)
		plt.plot(m, corr[m], 'o')
		plt.xticks([])
		plt.title('1/%d res; t=%.2f ms' % (i+1,1000*t), rotation=0)
		plt.ylim([-.1,.25])
		plt.gca().yaxis.set_label_position("right")

	plt.tight_layout()
	plt.show()