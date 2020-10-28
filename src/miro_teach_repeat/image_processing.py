#!/usr/bin/env python3

import sys
import cv_bridge
import cv2
import math
import yaml
import numpy as np
import numpy.lib
import scipy.signal
import image_geometry
from sensor_msgs.msg import CameraInfo

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
	patch_half_size = [int((p-1)/2) for p in patch_size]
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

def xcorr_match_images(image, template_image, subsampling=1):
	image = np.pad(image, ((0,),(int(template_image.shape[1]/2),)), mode='constant', constant_values=0)
	if subsampling == 1:
		corr = normxcorr2_horizontal_sweep(image, template_image)[0]
	else:
		corr = normxcorr2_subpixel(image, template_image, subsampling)
	offset = np.argmax(corr)
	return offset - (len(corr)-1)/2, corr[offset]

def xcorr_match_images_debug(image, template_image, subsampling=1):
	image_pad = np.pad(image, ((0,),(int(template_image.shape[1]/2),)), mode='constant', constant_values=0)
	if subsampling == 1:
		corr = normxcorr2_horizontal_sweep(image_pad, template_image)[0]
	else:
		corr = normxcorr2_subpixel(image_pad, template_image, subsampling)
	offset = np.argmax(corr)
	debug_image = create_correlation_debug_image(image, template_image, corr)
	return offset - (len(corr)-1)/2, corr[offset], debug_image

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
	stitched_image[:,-image_right.shape[1]:] += image_right * np.flip(blend_map_linear, 0)

	stitched_image = np.asarray(stitched_image, image_left.dtype) # get the same type out as we put in

	return stitched_image

def stitch_stereo_image_message(msg_left, msg_right, compressed=False):
	if compressed:
		return stitch_stereo_image(compressed_msg_to_image(msg_left),compressed_msg_to_image(msg_right))
	else:
		return stitch_stereo_image(msg_to_image(msg_left),msg_to_image(msg_right))

def rectify_stitch_stereo_image_message(msg_left, msg_right, info_left, info_right, compressed=False):
	if compressed:
		return rectify_stitch_stereo_image(compressed_msg_to_image(msg_left),compressed_msg_to_image(msg_right), info_left, info_right)
	else:
		return rectify_stitch_stereo_image(msg_to_image(msg_left),msg_to_image(msg_right), info_left, info_right)

def normxcorr2_horizontal_sweep(image_ref, image_query):
	# we only implement 'valid' mode
	# and only for horizontal sweeping
	assert(image_ref.shape[0] == image_query.shape[0])

	# make sure images are all positive
	image_ref = image_ref - image_ref.min() if image_ref.min() < 0 else image_ref.copy()
	image_query = image_query - image_query.min() if image_query.min() < 0 else image_query.copy()

	image_ref_zero_mean = image_ref - image_ref.mean()
	image_query_zero_mean = image_query - image_query.mean()

	xcorr = scipy.signal.correlate2d(image_ref_zero_mean, image_query_zero_mean, mode='valid')

	numerator = xcorr
	I_hat_d = running_horizontal_sum_patch(image_ref, image_query.shape[1])
	I_squared = running_horizontal_sum_patch(image_ref**2, image_query.shape[1])
	denominator = np.sqrt((I_squared - I_hat_d**2/image_query.size) * (image_query_zero_mean**2).sum())

	return numerator / denominator

def subpixel_shift_approx(img, x=0, y=0):
	if x != 0:
		intX = int(math.floor(x))
		if intX >= 1:
			img = np.hstack((np.zeros((img.shape[0],intX)), img[:,:-intX]))
		elif intX <= -1:
			img = np.hstack((img[:,-intX:], np.zeros((img.shape[0],-intX))))
		x = x % 1
		img = (1-x) * img + x * np.hstack((np.zeros((img.shape[0], 1)), img[:,:-1]))
	if y != 0:
		intY = int(math.floor(y))
		if intY >= 1:
			img = np.vstack((np.zeros((intY,img.shape[1])), img[:-intY,:]))
		elif intY <= -1:
			img = np.vstack((img[-intY:,:], np.zeros((-intY,img.shape[1]))))
		y = y % 1
		img = (1-y) * img + y * np.vstack((np.zeros((1,img.shape[1])), img[:-1,:]))
	
	return img

def normxcorr2_subpixel(image, template, subsamples):
	if subsamples < 1:
		subsamples = 1
	interp = np.arange(0, 1, 1./subsamples)
	corrs = []

	for i in interp:
		# image_interp = np.float64(subpixel_shift_approx(image,-i))
		image_interp = np.fft.ifft2(scipy.ndimage.fourier_shift(np.fft.fft2(image), (0, -i))).real
		corrs.append(normxcorr2_horizontal_sweep(image_interp, template))

	out = np.array(corrs).flatten('F')
	if subsamples > 1:
		out = out[:-subsamples+1]

	return out

def normxcorr2_subpixel_fast(image, template, subsamples):
	image_pad = np.pad(image, ((0,),(int(template.shape[1]/2),)), mode='constant', constant_values=0)
	basic_corr = normxcorr2_horizontal_sweep(image_pad, template)[0]
	best_index = np.argmax(basic_corr)
	best_offset = best_index - (len(basic_corr)-1)/2
	best_corr = basic_corr[best_index]

	if subsamples > 1:
		interp = np.arange(best_offset-1, best_offset+1, 1./subsamples)
		corrs = []

		for i in interp:
			image_interp = np.float64(subpixel_shift_approx(image, -i))
			# image_interp = np.fft.ifft2(scipy.ndimage.fourier_shift(np.fft.fft2(image), (0, -i))).real
			corrs.append(normxcorr2_horizontal_sweep(image_interp, template))

		best_index = np.argmax(corrs)
		best_offset = interp[best_index]
		best_corr = corrs[best_index]

	return best_offset, best_corr
	
def running_horizontal_sum_patch(image, patch_width):
	cumsum = image.sum(axis=0).cumsum()
	return cumsum[patch_width-1:] - cumsum[:-patch_width+1]

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
	if len(corr)-1 > img2.shape[1]:
		subsampling = (len(corr)-1) / (img2.shape[1]-1)
		corr = corr[::subsampling]

	offset = np.argmax(corr) - int(img2.shape[1]/2)
	debug_size = 50

	corr_positions = np.flip(np.int32(-(debug_size-1)*np.clip(corr,0,1)), 0) - 1

	debug_image = np.concatenate((img2, img1, -1*np.ones((debug_size,img1.shape[1]))), axis=0)
	debug_image = np.uint8(255.0 * (1 + debug_image) / 2.0)
	debug_image = cv2.merge((debug_image, debug_image, debug_image))

	# if the image width is even, we'll get one more correlation value than the image width
	if corr_positions.size > debug_image.shape[1]:
		debug_image = np.hstack((debug_image, np.zeros((debug_image.shape[0],corr_positions.size-debug_image.shape[1],3), dtype=np.uint8)))

	cv2.line(debug_image, (int(-offset+img1.shape[1]/2),0), (int(-offset+img1.shape[1]/2),img1.shape[0]-1), (0,255,0))
	cv2.line(debug_image, (int(img1.shape[1]/2),img1.shape[0]), (int(img1.shape[1]/2), 2*img1.shape[0]-1), (255,0,0))

	cv2.line(debug_image, (0,debug_image.shape[0]-int(debug_size/4)), (img1.shape[1],debug_image.shape[0]-int(debug_size/4)), (60,0,0))
	cv2.line(debug_image, (0,debug_image.shape[0]-int(debug_size/2)), (img1.shape[1],debug_image.shape[0]-int(debug_size/2)), (60,0,0))
	cv2.line(debug_image, (0,debug_image.shape[0]-int(3*debug_size/4)), (img1.shape[1],debug_image.shape[0]-int(3*debug_size/4)), (60,0,0))
	cv2.line(debug_image, (int(img1.shape[1]/2),2*img1.shape[0]), (int(img1.shape[1]/2),debug_image.shape[0]), (60,0,0))

	debug_image[corr_positions,np.arange(corr_positions.size),:] = 255
	return debug_image

def rectify_image(cam, img, size, offset=0):
	mapx, mapy = cv2.initUndistortRectifyMap(cam.K, cam.D, cam.R, cam.P, size[::-1], cv2.CV_32FC1)
	return cv2.remap(img, mapx, mapy, cv2.INTER_CUBIC), mapx

def read_file(filename):
	with open(filename,'r') as file:
		data = file.read()
	return data

def yaml_to_camera_info(yaml_data):
	camera_info = CameraInfo()
	camera_info.header.frame_id = yaml_data['camera_name']
	camera_info.D = yaml_data['distortion_coefficients']['data']
	camera_info.K = yaml_data['camera_matrix']['data']
	camera_info.P = yaml_data['projection_matrix']['data']
	camera_info.R = yaml_data['rectification_matrix']['data']
	camera_info.distortion_model = yaml_data['distortion_model']
	camera_info.height = yaml_data['image_height']
	camera_info.width = yaml_data['image_width']
	return camera_info

def camera_info_to_yaml(camera_info):
	yaml_data = {}
	yaml_data['camera_name'] = camera_info.header.frame_id
	yaml_data['distortion_coefficients'] = {'data':camera_info.D,'cols':5,'rows':1}
	yaml_data['camera_matrix'] = {'data':camera_info.K,'cols':3,'rows':3}
	yaml_data['projection_matrix'] = {'data':camera_info.P,'cols':4,'rows':3}
	yaml_data['rectification_matrix'] = {'data':camera_info.R,'cols':3,'rows':3}
	yaml_data['distortion_model'] = camera_info.distortion_model
	yaml_data['image_height'] = camera_info.height
	yaml_data['image_width'] = camera_info.width
	return yaml_data

def rectify_stitch_stereo_image(image_left, image_right, info_left, info_right):
	cam_left = image_geometry.PinholeCameraModel()
	cam_left.fromCameraInfo(info_left)
	cam_right = image_geometry.PinholeCameraModel()
	cam_right.fromCameraInfo(info_right)

	if len(image_left.shape) > 2 and image_left.shape[2] > 1:
		image_left = grayscale(image_left)
		image_right = grayscale(image_right)

	extra_space = 200
	cam_left.P[0,2] += extra_space

	warped_image_size = (image_left.shape[0],image_left.shape[1]+extra_space)
	warped_left, left_mapx = rectify_image(cam_left, image_left, warped_image_size)
	warped_right, right_mapx = rectify_image(cam_right, image_right, warped_image_size)
	stitched = np.zeros((warped_left.shape[0],warped_left.shape[1]+extra_space))
	non_overlap_pixels = extra_space+200
	blank_pixels = 200
	overlap_pixels = warped_left.shape[1] - non_overlap_pixels - blank_pixels
	blend_map_linear = np.concatenate((np.ones(non_overlap_pixels),np.arange(1,0,-1.0/(overlap_pixels+1))[1:],np.zeros(blank_pixels)))
	stitched[:,:warped_left.shape[1]] = warped_left * blend_map_linear
	stitched[:,-warped_right.shape[1]:] += warped_right * np.flip(blend_map_linear, 0)
	
	stitched = np.asarray(stitched, image_left.dtype) # get the same type out as we put in

	fov = np.zeros((stitched.shape[1]))
	fl = np.linspace(60.6+27,-60.6+27,image_left.shape[1]).reshape(1,-1)
	fr = np.linspace(60.6-27,-60.6-27,image_right.shape[1]).reshape(1,-1)
	left_mapx = left_mapx.mean(axis=0)
	right_mapx = right_mapx.mean(axis=0)
	fov_l = cv2.remap(fl, left_mapx, np.zeros_like(left_mapx), cv2.INTER_CUBIC).flatten()
	fov_r = cv2.remap(fr, right_mapx, np.zeros_like(right_mapx), cv2.INTER_CUBIC).flatten()
	fov[:fov_l.size] = fov_l * blend_map_linear
	fov[-fov_r.size:] += fov_r * np.flip(blend_map_linear, 0)

	return stitched, fov

def parse_patch_size_parameter(patch_size):
	if type(patch_size) == tuple:
		return patch_size
	if type(patch_size) == str:
		return tuple([int(sz) for sz in patch_size[1:-1].split(',')])
	elif type(patch_size) == int:
		return (patch_size, patch_size)