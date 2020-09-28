import os
import json
import math
import numpy as np
from rospy_message_converter import message_converter
from geometry_msgs.msg import Pose
import tf_conversions
import matplotlib.pyplot as plt
from tqdm import tqdm
import scipy.optimize
import cv2

def read_file(filename):
	with open(filename, 'r') as f:
		data = f.read()
	return data

def get_ground_truth_poses(dir):
	pose_files = np.array([f for f in os.listdir(dir) if f.endswith('_map_to_base_link.txt')])
	nums = [int(s.split('_')[0]) for s in pose_files]
	idx = np.argsort(nums)
	pose_files = [dir+f for f in pose_files[idx]]
	transforms = [message_converter.convert_dictionary_to_ros_message('geometry_msgs/TransformStamped',json.loads(read_file(p))).transform for p in pose_files]
	return [tf_conversions.fromMsg(Pose(tf.translation,tf.rotation)) for tf in transforms]

def get_pose_x_y_theta(poses):
	x = np.array([pose.p.x() for pose in poses])
	y = np.array([pose.p.y() for pose in poses])
	theta = np.array([pose.M.GetRPY()[2] for pose in poses])
	return np.vstack((x, y, theta))

def sample_repeats_to_teach(teach, repeats):
	'''Make the repeats the same length as teach, using the closest possible poses'''
	repeats_sampled = [np.full(teach.shape, np.nan) for r in repeats]
	window_half_size = 0.2 # search for match in corresponding 2*window_half_size of repeat run
	angle_threshold = math.radians(30) # make sure the matching poses are facing the same direction

	# HUNGARIAN ASSIGNMENT
	MAX = 10000000
	teach_length = teach.shape[1]
	for n_repeat, repeat in enumerate(repeats):
		repeat_length = repeat.shape[1]
		cost = np.full((teach_length, repeat_length), MAX)

		for i in range(teach_length):
			range_min = max(0, int(round((float(i) / teach_length - window_half_size) * repeat_length)))
			range_max = min(repeat_length, int(round((float(i) / teach_length + window_half_size) * repeat_length)))
			r_window = repeat[:,range_min:range_max]
			dist = (r_window[0,:] - teach[0,i])**2 + (r_window[1,:] - teach[1,i])**2
			dist[abs(wrapToPi(r_window[2,:] - teach[2,i])) > angle_threshold] = MAX

			cost[i,range_min:range_max] = dist

		teach_ind, repeat_ind = scipy.optimize.linear_sum_assignment(cost)
		repeats_sampled[n_repeat][:,teach_ind] = repeat[:,repeat_ind]
	
	return repeats_sampled

def rotation_matrix(rad):
	return np.array(((math.cos(rad),-math.sin(rad)),(math.sin(rad),math.cos(rad))))

def get_repeat_errors(teach, repeats):
	'''With matching length teach and repeat runs, get the position error parallel and perpendicular to the teach path'''
	path_errors = [np.zeros((teach.shape[1])) for r in repeats]
	lateral_errors = [np.zeros((teach.shape[1])) for r in repeats]
	orientation_errors = [np.zeros((teach.shape[1])) for r in repeats]

	for i in range(teach.shape[1]):
		teach_pos = teach[:2,i]
		teach_angle = teach[2,i]
		for j, r in enumerate(repeats):
			repeat_pos = r[:2,i]
			repeat_angle = r[2,i]
			pos_error = teach_pos - repeat_pos
			rotated_error = rotation_matrix(-teach_angle).dot(pos_error)
			path_errors[j][i] = rotated_error[0]
			lateral_errors[j][i] = rotated_error[1]
			orientation_errors[j][i] = wrapToPi(teach_angle - repeat_angle)
	return path_errors, lateral_errors, orientation_errors

def wrapToPi(x):
	'''wrap angle to between +pi and -pi'''
	return ((x + math.pi) % (2*math.pi)) - math.pi

def quiver_plot(poses, colour):
	plt.quiver(poses[0], poses[1], np.cos(poses[2]), np.sin(poses[2]), color=colour, scale=50)

def line_plot(poses, colour):
	plt.plot(poses[0], poses[1], color=colour)

teach_run = os.path.expanduser('/media/dominic/NewVolume/teach-repeat/teach-repeat-data/2020-09-17_13:27:47/')
repeat_runs = ['/media/dominic/NewVolume/teach-repeat/teach-repeat-data/2020-09-17_13:27:47-repeat-1.0-1/','/media/dominic/NewVolume/teach-repeat/teach-repeat-data/2020-09-17_13:27:47-repeat-1.0-2/']
# teach_run = os.path.expanduser('/media/dominic/NewVolume/teach-repeat/teach-repeat-data/2020-09-15_07_00_47/')
# repeat_runs = [
# 	'/media/dominic/NewVolume/teach-repeat/teach-repeat-data/2020-09-15_07_00_47-repeat-0.7-1/',
# 	'/media/dominic/NewVolume/teach-repeat/teach-repeat-data/2020-09-15_07_00_47-repeat-0.8-1/',
# 	'/media/dominic/NewVolume/teach-repeat/teach-repeat-data/2020-09-15_07_00_47-repeat-0.9-2/',
# 	'/media/dominic/NewVolume/teach-repeat/teach-repeat-data/2020-09-15_07_00_47-repeat-1.0-1/',
# 	'/media/dominic/NewVolume/teach-repeat/teach-repeat-data/2020-09-15_07_00_47-repeat-1.1-1/',
# 	'/media/dominic/NewVolume/teach-repeat/teach-repeat-data/2020-09-15_07_00_47-repeat-1.2-1/',
# 	# '/media/dominic/NewVolume/teach-repeat/teach-repeat-data/2020-09-15_07_00_47-repeat-1.3-1/',
# ]
# repeat_runs = [os.path.expanduser(path) for path in repeat_runs]

teach_poses = get_pose_x_y_theta(get_ground_truth_poses(teach_run))
repeat_poses = [get_pose_x_y_theta(get_ground_truth_poses(repeat_dir)) for repeat_dir in repeat_runs]

colours = ['#44dd44', '#4444dd', '#dd4444', '#dddd44','#dd44dd','#44dddd','#888888','#2288dd']

SHOW_DATA_FOR_CROPPING = False
if SHOW_DATA_FOR_CROPPING:
	fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
	for pose_data_list,colour in zip(repeat_poses,colours[1:]):
		ax1.plot(pose_data_list[0], color=colour)
		ax2.plot(pose_data_list[1], color=colour)
		ax3.plot(pose_data_list[2], color=colour)
	ax1.set_title('Repeat runs')
	ax1.set_ylabel('x (m)')
	ax2.set_ylabel('y (m)')
	ax3.set_ylabel('theta (rad)')
	plt.legend([str(num) for num in list(range(len(repeat_poses)))])
	plt.show()
# crop to remove time spent standing still from repeat runs
# repeat_poses[1] = repeat_poses[1][:,:1300]
# repeat_poses[2] = repeat_poses[2][:,300:1800]
# filter errors due to SLAM relocalisation in teach run
teach_diff = np.sqrt(np.sum((teach_poses[:2,1:] - teach_poses[:2,:-1]) ** 2, axis=0))
bad_teach_poses = np.hstack((np.full((1,),False),teach_diff >= 0.5))

# match the size of repeat runs to the teach run (bearnav is sampled more often)
# repeat_poses = [np.hstack((repeat_p,np.full((3,teach_poses.shape[1]-repeat_p.shape[1]),np.nan))) for repeat_p in repeat_poses]
# repeat_poses = sample_repeats_to_teach(teach_poses, repeat_poses)
# repeat_errors_path, repeat_errors_lateral, repeat_errors_orientation = get_repeat_errors(teach_poses, repeat_poses)

# filter errors due to SLAM relocalisation in repeat runs
# for i in range(len(repeat_errors_path)):
# 	repeat_diff = np.sqrt(np.sum((repeat_poses[i][:2,1:] - repeat_poses[i][:2,:-1]) ** 2, axis=0))
# 	bad_repeat_poses = np.hstack((np.full((1,),False),repeat_diff >= 0.5)) | bad_teach_poses
# 	repeat_errors_path[i][bad_repeat_poses] = np.nan
# 	repeat_errors_lateral[i][bad_repeat_poses] = np.nan
# 	repeat_errors_orientation[i][bad_repeat_poses] = np.nan

	# bad
	# repeat_errors_lateral[i][abs(repeat_errors_lateral[i]) > 5] = np.nan


# RMS_lateral = [math.sqrt(np.nanmean(errors_lateral**2)) for errors_lateral in repeat_errors_lateral]
# RMS_orientation = [math.sqrt(np.nanmean(errors_orientation**2)) for errors_orientation in repeat_errors_orientation]


# Show overview of the runs
plt.figure()
img = cv2.imread('/home/dominic/Desktop/outdoor3longands4.pgm')
img_extent = (-32.270855, img.shape[0]*0.05-32.270855, -149.629161, img.shape[1]*0.05-149.629161)
plt.imshow(img, extent=img_extent)
quiver_plot(teach_poses, colours[0])
for pose_data_list,colour in zip(repeat_poses,colours[1:]):
	quiver_plot(pose_data_list, colour)
	# x_link = np.vstack((teach_poses[0],pose_data_list[0]))
	# y_link = np.vstack((teach_poses[1],pose_data_list[1]))
	# plt.plot(x_link,y_link)
plt.title('overview of runs')
# plt.legend(['teach'] + [("odom x %.1f" % x) for x in np.arange(0.7,1.3,0.1)])

# Show the lateral path error
# along path error doesn't make sense because the repeat run is sampled more frequently (for bearnav)
# so the along path error will always be tiny.
# fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
# for err_lateral,err_orientation,colour in zip(repeat_errors_lateral,repeat_errors_orientation,colours[1:]):
# 	ax1.plot(err_lateral, color=colour)
# 	ax2.plot(err_orientation, color=colour)
# ax1.set_title('Repeat run error')
# ax1.set_ylabel('lateral path error (m)')
# ax2.set_ylabel('orientation path error (rad)')
# ax1.legend(['RMS=%f m' % RMS_lat for RMS_lat in RMS_lateral])
# ax2.legend(['RMS=%f rad' % RMS_ori for RMS_ori in RMS_orientation])
plt.show()