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
		image = cv2.resize(image, resize, interpolation=cv2.INTER_NEAREST)
	return patch_normalise_pad(image, patch_size)

def patch_normalise_image(image, patch_size, resize=None):
	# resize should be height * width
	if len(image.shape) > 2 and image.shape[2] > 1:
		image = grayscale(image)
	if resize is not None:
		# opencv sizes are the opposite of numpy (width*height)
		resize = tuple(reversed(resize))
		image = cv2.resize(image, resize, interpolation=cv2.INTER_NEAREST)
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
	image = image.copy()
	template_image = template_image.copy()
	image = np.pad(image, ((0,),(int(template_image.shape[1]/2),)), mode='constant', constant_values=0)
	corr = normxcorr2(image, template_image, mode='valid')
	offset = np.argmax(corr)
	return offset - int(template_image.shape[1]/2), corr[0,offset]

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

def normxcorr2(image, template, mode="full"):
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



if __name__ == "__main__":
	np.random.seed(0)
	# img = np.float64(np.random.randint(0,256,(44,115), dtype=np.uint8))
	img1 = grayscale(cv2.imread('/home/dominic/Pictures/L2.png'))
	img2 = grayscale(cv2.imread('/home/dominic/Pictures/R2.png'))

	import image_geometry
	import scipy.spatial.transform

	height, width = img1.shape
	K = np.array([339.5929335036774, 0.0, width/2, 0.0, 338.1646422931829, height/2, 0.0, 0.0, 1.0]).reshape(3,3)
	# K1 = np.array([339.5929335036774, 0.0, 302.3151446150193, 0.0, 338.1646422931829, 164.24615236157783, 0.0, 0.0, 1.0]).reshape(3,3)
	# K2 = np.array([421.8457462857705, 0.0, 307.19951192683885, 0.0, 420.2160629941844, 139.56714513486457, 0.0, 0.0, 1.0]).reshape(3,3)
	R1 = scipy.spatial.transform.Rotation.from_euler('Y',math.radians(-27)).as_dcm()
	R2 = scipy.spatial.transform.Rotation.from_euler('Y',math.radians(27)).as_dcm()
	D1 = np.array([-0.31355715866352996, 0.07348274729264032, 0.0013916779653480483, 0.0010994248771950602, 0.0])
	D2 = np.array([-0.38126443041527774, 0.1004896256759518, 0.01632508148069258, 0.003089452847769042, 0.0])

	OUT_WIDTH = width
	OUT_HEIGHT = height

	k1 = cv2.getOptimalNewCameraMatrix(K,D1,(width, height),1.0,(OUT_WIDTH,OUT_HEIGHT))
	P1 = k1[0]
	k2 = cv2.getOptimalNewCameraMatrix(K,D2,(width, height),1.0,(OUT_WIDTH,OUT_HEIGHT))
	P2 = k2[0]

	mapx1, mapy1 = cv2.initUndistortRectifyMap(K, D1, R1, P1, (OUT_WIDTH, OUT_HEIGHT), cv2.CV_32FC1)
	mapx2, mapy2 = cv2.initUndistortRectifyMap(K, D2, R2, P2, (OUT_WIDTH, OUT_HEIGHT), cv2.CV_32FC1)
	warp1 = cv2.remap(img1, mapx1, mapy1, cv2.INTER_CUBIC)
	warp2 = cv2.remap(img2, mapx2, mapy2, cv2.INTER_CUBIC)

	# warp1 = warp1[max(k1[1][1],k2[1][1]):min(k1[1][3],k2[1][3]),:]
	# warp2 = warp2[max(k1[1][1],k2[1][1]):min(k1[1][3],k2[1][3]),:]

	non_overlap_pixels = 200
	overlap_pixels = OUT_WIDTH - non_overlap_pixels*2
	blend_map_linear = np.concatenate((np.ones(non_overlap_pixels),np.arange(1,0,-1.0/(overlap_pixels+1))[1:],np.zeros(non_overlap_pixels)))
	combined = np.uint8(blend_map_linear * warp1 + np.flip(blend_map_linear) * warp2)

	combined2 = np.uint8(stitch_stereo_image(img1, img2))

	cv2.imshow('L', warp1)
	cv2.imshow('R', warp2)
	cv2.imshow('both1', combined)
	cv2.imshow('both2', combined2)
	cv2.waitKey()
	
	# import time
	# t = time.time()
	# print(image_scanline_rotation(img1[40:80,140:-140], img2[40:80,140:-140], 463//2 - 140))
	# print(image_patch_rotation(img1[40:80,140:-140], img2[40:80,140:-140], 463//2 - 140))
	# print(xcorr_match_images(img1[40:80,140:-140], img2[40:80,140:-140]))
	# print('%fs' % (time.time() - t))