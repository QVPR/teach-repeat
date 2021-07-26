#!/usr/bin/env python

import numpy as np
import cv2
import os
import pickle
import time
import json
import math
import sys
import tf_conversions
from rospy_message_converter import message_converter
import matplotlib.pyplot as plt
import matplotlib.style
import warnings
import matplotlib.lines as mlines
from sklearn import linear_model

sys.path.append(os.path.dirname(__file__) + '/../nodes')
import importlib
image_processing = importlib.import_module('image_processing')

def read_file(filename):
	with open(filename, 'r') as f:
		data = f.read()
	return data

def read_pose_files(pose_files):
	return [tf_conversions.fromMsg(message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',json.loads(read_file(p)))) for p in pose_files]
	
def read_image_files(image_files):
	return [pickle.loads(read_file(img)) for img in image_files]

# Stole this from matplotlib3 source code...
def legend_elements(self, prop="colors", num="auto",
					 fmt=None, func=lambda x: x, **kwargs):
		"""
		Creates legend handles and labels for a PathCollection. This is useful
		for obtaining a legend for a :meth:`~.Axes.scatter` plot. E.g.::

			scatter = plt.scatter([1, 2, 3],  [4, 5, 6],  c=[7, 2, 3])
			plt.legend(*scatter.legend_elements())

		Also see the :ref:`automatedlegendcreation` example.

		Parameters
		----------
		prop : string, optional, default *"colors"*
			Can be *"colors"* or *"sizes"*. In case of *"colors"*, the legend
			handles will show the different colors of the collection. In case
			of "sizes", the legend will show the different sizes.
		num : int, None, "auto" (default), array-like, or `~.ticker.Locator`,
			optional
			Target number of elements to create.
			If None, use all unique elements of the mappable array. If an
			integer, target to use *num* elements in the normed range.
			If *"auto"*, try to determine which option better suits the nature
			of the data.
			The number of created elements may slightly deviate from *num* due
			to a `~.ticker.Locator` being used to find useful locations.
			If a list or array, use exactly those elements for the legend.
			Finally, a `~.ticker.Locator` can be provided.
		fmt : str, `~matplotlib.ticker.Formatter`, or None (default)
			The format or formatter to use for the labels. If a string must be
			a valid input for a `~.StrMethodFormatter`. If None (the default),
			use a `~.ScalarFormatter`.
		func : function, default *lambda x: x*
			Function to calculate the labels. Often the size (or color)
			argument to :meth:`~.Axes.scatter` will have been pre-processed
			by the user using a function *s = f(x)* to make the markers
			visible; e.g. *size = np.log10(x)*. Providing the inverse of this
			function here allows that pre-processing to be inverted, so that
			the legend labels have the correct values;
			e.g. *func = np.exp(x, 10)*.
		kwargs : further parameters
			Allowed keyword arguments are *color* and *size*. E.g. it may be
			useful to set the color of the markers if *prop="sizes"* is used;
			similarly to set the size of the markers if *prop="colors"* is
			used. Any further parameters are passed onto the `.Line2D`
			instance. This may be useful to e.g. specify a different
			*markeredgecolor* or *alpha* for the legend handles.

		Returns
		-------
		tuple (handles, labels)
			with *handles* being a list of `.Line2D`  objects
			and *labels* a matching list of strings.
		"""
		handles = []
		labels = []
		hasarray = self.get_array() is not None
		if fmt is None:
			fmt = matplotlib.ticker.ScalarFormatter(useOffset=False, useMathText=True)
		elif isinstance(fmt, str):
			fmt = matplotlib.ticker.StrMethodFormatter(fmt)
		fmt.create_dummy_axis()

		if prop == "colors":
			if not hasarray:
				warnings.warn("Collection without array used. Make sure to "
							  "specify the values to be colormapped via the "
							  "`c` argument.")
				return handles, labels
			u = np.unique(self.get_array())
			size = kwargs.pop("size", matplotlib.rcParams["lines.markersize"])
		elif prop == "sizes":
			u = np.unique(self.get_sizes())
			color = kwargs.pop("color", "k")
		else:
			raise ValueError("Valid values for `prop` are 'colors' or "
							 "'sizes'. You supplied '"+str(prop)+"' instead.")

		fmt.set_bounds(func(u).min(), func(u).max())
		if num == "auto":
			num = 9
			if len(u) <= num:
				num = None
		if num is None:
			values = u
			label_values = func(values)
		else:
			if prop == "colors":
				arr = self.get_array()
			elif prop == "sizes":
				arr = self.get_sizes()
			if isinstance(num, matplotlib.ticker.Locator):
				loc = num
			elif np.iterable(num):
				loc = matplotlib.ticker.FixedLocator(num)
			else:
				num = int(num)
				loc = matplotlib.ticker.MaxNLocator(nbins=num, min_n_ticks=num-1,
											 steps=[1, 2, 2.5, 3, 5, 6, 8, 10])
			label_values = loc.tick_values(func(arr).min(), func(arr).max())
			cond = ((label_values >= func(arr).min()) &
					(label_values <= func(arr).max()))
			label_values = label_values[cond]
			xarr = np.linspace(arr.min(), arr.max(), 256)
			values = np.interp(label_values, func(xarr), xarr)

		kw = dict(markeredgewidth=self.get_linewidths()[0],
				  alpha=self.get_alpha())
		kw.update(kwargs)

		for val, lab in zip(values, label_values):
			if prop == "colors":
				color = self.cmap(self.norm(val))
			elif prop == "sizes":
				size = np.sqrt(val)
				if np.isclose(size, 0.0):
					continue
			h = mlines.Line2D([0], [0], ls="", color=color, ms=size,
							  marker=self.get_paths()[0], **kw)
			handles.append(h)
			if hasattr(fmt, "set_locs"):
				fmt.set_locs(label_values)
			l = fmt(lab)
			labels.append(l)

		return handles, labels

def calculate_image_offsets(base_image, images, poses):
	thetas = np.zeros(len(images))
	offsets = np.zeros((4,len(images)))
	correlations = np.zeros((4,len(images)))

	for i,(image,pose) in enumerate(zip(images,poses)):
		thetas[i] = math.degrees(pose.M.GetRPY()[2])

		crop_amount = 80
		half_crop = crop_amount // 2

		# stitched_img = np.uint8(255.0/2 * (1+np.vstack((base_image, np.zeros((10,115))-.5,base_image,np.zeros((10,115))-.5,base_image))))
		# stitched_img[44+10:2*44+10,-crop_amount:half_crop] = 64
		# stitched_img[44+10:2*44+10,-half_crop:crop_amount] = 64

		# stitched_img[2*44+2*10:,:] = 64
		# stitched_img[2*44+2*10:2*44+2*10+20,half_crop:-half_crop] = stitched_img[:20,half_crop:-half_crop]
		# stitched_img[-20:,half_crop:-half_crop] = stitched_img[44-20:44,half_crop:-half_crop]

		# cv2.imshow('img', stitched_img)
		# cv2.waitKey()

		offsets[0,i], correlations[0,i] = image_processing.xcorr_match_images(base_image, image)
		offsets[1,i], correlations[1,i] = image_processing.xcorr_match_images(base_image, image[:,:-crop_amount])
		
		offsets[2,i], correlations[2,i] = image_processing.xcorr_match_images(base_image, image[:,half_crop:-half_crop])
		# offsets[2,i], correlations[2,i] = image_processing.xcorr_match_images(base_image[:20,:], image[:20,half_crop:-half_crop])
		# offsets[2,i], correlations[2,i] = image_processing.xcorr_match_images(base_image[-20:,:], image[-20:,half_crop:-half_crop])
		offsets[2,i] -= half_crop

		# offsets[3,i], correlations[3,i] = image_processing.xcorr_match_images(base_image[:20,:], image[:20,half_crop:-half_crop])
		# offsets[3,i] -= half_crop

		offsets[3,i], correlations[3,i] = image_processing.xcorr_match_images(base_image, image[:,crop_amount:])
		offsets[3,i] -= crop_amount
	return (thetas, offsets, correlations)

dirs = [os.path.expanduser('~/miro/data/imtest') + ending + '/' for ending in ['M','L','R']]
base_image = pickle.loads(read_file(os.path.expanduser('~/miro/data/imtestM/000000_image.pkl')))

files_n = [range(len(os.listdir(dir)) // 4) for dir in dirs]

images = [read_image_files([dir + ("%06d_image.pkl" % i) for i in files_n[d]]) for d,dir in enumerate(dirs)]
poses = [read_pose_files([dir + ("%06d_pose.txt" % i) for i in files_n[d]]) for d,dir in enumerate(dirs)]

thetas = [[] for d in dirs]
offsets = [[] for d in dirs]
correlations = [[] for d in dirs]

for i in range(len(dirs)):
	thetas[i], offsets[i], correlations[i] = calculate_image_offsets(base_image, images[i], poses[i])
	thetas[i] -= thetas[i][0] # stuffed up starting theta at 0 in one test - normalise


matplotlib.style.use('ggplot')
fig, ax = plt.subplots()
fig.set_size_inches((9.6,7.2))
t = 0
sz = 60
def plot_scatter(thetas,offsets,corrs,c):
	return ax.scatter(thetas[abs(corrs)>t], -offsets[abs(corrs)>t], s=sz*corrs[abs(corrs)>t], alpha=0.7, color=c)
n = 0
# Scatter offset vs theta
sc1 = plot_scatter(thetas[0], offsets[0][n,:], correlations[0][n,:], 'r')
sc2 = plot_scatter(thetas[1], offsets[1][n,:], correlations[1][n,:], 'g')
sc3 = plot_scatter(thetas[2], offsets[2][n,:], correlations[2][n,:], 'b')
ax.plot(0,0,'+',color='#000000')
m1 = linear_model.RANSACRegressor(residual_threshold=3)
m2 = linear_model.RANSACRegressor(residual_threshold=3)
m3 = linear_model.RANSACRegressor(residual_threshold=3)
m1.fit(thetas[0][abs(thetas[0]) < 30].reshape(-1, 1), -offsets[0][n,:][abs(thetas[0]) < 30])
m2.fit(thetas[1][abs(thetas[1]) < 30].reshape(-1, 1), -offsets[1][n,:][abs(thetas[1]) < 30])
m3.fit(thetas[2][abs(thetas[2]) < 30].reshape(-1, 1), -offsets[2][n,:][abs(thetas[2]) < 30])
ax.plot([-30,30],m1.predict([[-30],[30]]),'r', alpha=0.5)
ax.plot([-30,30],m2.predict([[-30],[30]]),'g', alpha=0.5)
ax.plot([-30,30],m3.predict([[-30],[30]]),'b', alpha=0.5)
ax.set_xlabel(r'offset $\theta$ ($\degree$)')
ax.set_ylabel('image offset (px)')
ax.set_title('mid NCC: im=[115x44], T=[35x44]')
# ax.set_xlim([-40,40])
# ax.set_ylim([-40,40])

print(m1.estimator_.coef_, m1.estimator_.intercept_)
print(m2.estimator_.coef_, m2.estimator_.intercept_)
print(m3.estimator_.coef_, m3.estimator_.intercept_)

# compared top to bottom offset
# sc1 = plot_scatter(-offsets[0][3,:], offsets[0][n,:], correlations[0][n,:], 'r')
# sc2 = plot_scatter(-offsets[1][3,:], offsets[1][n,:], correlations[1][n,:], 'g')
# sc3 = plot_scatter(-offsets[2][3,:], offsets[2][n,:], correlations[2][n,:], 'b')
# ax.plot([-20,20],[-20,20],'k--', alpha=0.3)
# ax.set_xlabel('top offset (px)')
# ax.set_ylabel('bottom offset (px)')
# ax.set_title('top and bottom NCC: im=[115x20], T=[35x20]')
# ax.set_xlim([-30,30])
# ax.set_ylim([-30,30])


l1=ax.legend([sc1,sc2,sc3],['0 cm','27 cm','-27 cm'],loc='lower right',title='hor. offset')
# l1=ax.legend(['Middle','Left'],loc='center left',title='hor. offset')
l2=ax.legend(*legend_elements(sc1, prop="sizes", num=5, alpha=0.7, func=lambda x:x/sz, color='r'),title='NCC',loc='upper left')
ax.add_artist(l1)

# plt.figure()
# N = 0
# t = 0.1
# T = np.arange(len(thetas[N]))
# plt.plot(t,-thetas[1],'k-')
# plt.plot(T[correlations[N][0,:]>t],offsets[N][0,:][correlations[N][0,:]>t],'r.')
# plt.plot(T[correlations[N][1,:]>t],offsets[N][1,:][correlations[N][1,:]>t],'g.')
# plt.plot(T[correlations[N][3,:]>t],offsets[N][3,:][correlations[N][3,:]>t],'b.')
# plt.plot(offsets_mid[correlations_mid>0.2],'g.')
# plt.plot(offsets_left[correlations_left>0.2],'b.')
# plt.plot(offsets_right[correlations_right>0.2],'m.')

plt.show()