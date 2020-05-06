#!/usr/bin/python

# Note: tf_conversions.fromMsg(tf_conversions.toMsg(odom_msg)) !== odom_msg
# some accuracy is lost going from Q -> DCM -> Q

import tf_conversions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import math
import numpy as np
import matplotlib.pyplot as plt
from rospy_message_converter import message_converter
import json
import os

def wrapToPi(x):
	return ((x + math.pi) % (2*math.pi)) - math.pi

def read_file(filename):
	with open(filename, 'r') as f:
		data = f.read()
	return data

def get_pose_files(dir):
	pose_files = [dir+f for f in os.listdir(dir) if f[-9:] == '_pose.txt']
	pose_files.sort()
	return pose_files

def read_pose_files(pose_files):
	return [tf_conversions.fromMsg(message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',json.loads(read_file(p)))) for p in pose_files]

def get_pose_x_y_theta(poses):
	x = np.array([pose.p.x() for pose in poses])
	y = np.array([pose.p.y() for pose in poses])
	theta = np.array([pose.M.GetRPY()[2] for pose in poses])
	return x, y, theta

def rotation_matrix(rad):
	return np.array(((math.cos(rad),-math.sin(rad)),(math.sin(rad),math.cos(rad))));

def np_to_frame(pose_tuple):
	return tf_conversions.Frame(tf_conversions.Rotation.RotZ(pose_tuple[2]),tf_conversions.Vector(pose_tuple[0],pose_tuple[1],0))

def frame_to_np(pose_frame):
	return np.array((pose_frame.p.x(), pose_frame.p.y(), pose_frame.M.GetRPY()[2]))

MAX_V = 0.4
MAX_OMEGA = 2.439
dt = 0.02

gain_rho = 0.3
gain_alpha = 5.0
gain_beta = -3.0

np.random.seed(7)

directory = os.path.expanduser('~/miro/data/follow-long-path/')
pose_files = get_pose_files(directory)
poses = read_pose_files(pose_files)
pose_data = get_pose_x_y_theta(poses)

targets = np.hstack([data.reshape(-1,1) for data in pose_data])
targets = targets[:-2,:]
# targets = np.vstack((np.arange(0,40,0.2),np.zeros(200),np.zeros(200))).T
target_index = 0
target = targets[target_index]
target_to_navigate_to = target + np.array([0.1*math.cos(target[2]),0.1*math.sin(target[2]),0])

actual_targets = []
update_locations = []
x_errors = []

odom = Odometry()
odom.pose.pose.position.x = 0
odom.pose.pose.position.y = 0
q = tf_conversions.Frame().M.RotZ(math.radians( 0 )).GetQuaternion()
odom.pose.pose.orientation.z = q[2]
odom.pose.pose.orientation.w = q[3]
odom_frame = tf_conversions.fromMsg(odom.pose.pose)

ground_truth = Pose()
ground_truth.position.x = odom.pose.pose.position.x
ground_truth.position.y = 0.5#odom.pose.pose.position.y
ground_truth.orientation.w = odom.pose.pose.orientation.w
ground_truth.orientation.z = odom.pose.pose.orientation.z
ground_truth_frame = tf_conversions.fromMsg(ground_truth)

N = 12000
xs = np.zeros(N)
ys = np.zeros(N)
thetas = np.zeros(N)
gt_xs = np.zeros(N)
gt_ys = np.zeros(N)
gt_thetas = np.zeros(N)

# fig = plt.figure()
# ax = fig.gca()
# line_targets = ax.quiver(targets[:,0], targets[:,1], np.cos(targets[:,2]), np.sin(targets[:,2]))
# line_odom = ax.quiver([],[],[],[], scale=50, color='#00ff00', alpha = 0.5)
# line_gt = ax.quiver([],[],[],[], scale=50, color='#0000ff', alpha = 0.5)
# fig.canvas.draw()
# fig_bg = fig.canvas.copy_from_bbox(ax.bbox)
# plt.show(block=False)

for i in range(N):
	xs[i] = odom_frame.p.x()
	ys[i] = odom_frame.p.y()
	theta = odom_frame.M.GetRPY()[2]
	thetas[i] = theta

	gt_xs[i] = ground_truth_frame.p.x()
	gt_ys[i] = ground_truth_frame.p.y()
	gt_theta = ground_truth_frame.M.GetRPY()[2]
	gt_thetas[i] =  gt_theta

	target_pos = (target_to_navigate_to[0], target_to_navigate_to[1], 0)
	target_theta = target_to_navigate_to[2]

	d_pos = tf_conversions.Vector(*target_pos) - odom_frame.p

	rho = d_pos.Norm()
	alpha = wrapToPi(math.atan2(d_pos.y(), d_pos.x()) - theta)
	beta = wrapToPi(-theta - alpha + target_theta)

	v = gain_rho * rho
	omega = gain_alpha * alpha + gain_beta * beta

	if abs(v) > MAX_V or abs(omega) > MAX_OMEGA:
		turn_rate = abs(omega / v)
		turn_rate_at_max = MAX_OMEGA / MAX_V

		if turn_rate > turn_rate_at_max:
			omega = MAX_OMEGA * omega / abs(omega)
			v = MAX_OMEGA / turn_rate * v / abs(v)
		else:
			omega = MAX_V * turn_rate * omega / abs(omega)
			v = MAX_V * v / abs(v)

	# gt_target_offset = np_to_frame(targets[target_index]).Inverse() * ground_truth_frame
	# if abs(gt_target_offset.p.x()) < 0.05:
	if rho < 0.10:
		target_index += 1
		if target_index == len(targets):
			target_index = 0

		current_target = np_to_frame(target)
		current_target_gt = np_to_frame(targets[target_index-1])
		new_target_gt = np_to_frame(targets[target_index])
		
		# current_target_gt -> new_target_gt in current_target_gt frame (x = forwards)
		target_offset = current_target_gt.Inverse() * new_target_gt 
		# transform to odom frame (x = positive odom) [same frame as current_target]
		target_offset.p = tf_conversions.Rotation.RotZ(current_target.M.GetRPY()[2]) * target_offset.p
		#Note: theoeretically this should be part of this?
		# target_offset.p += np_to_frame(d_pos).p

		# get the offset from the current target (simulates image offset)
		gt_target_offset = current_target_gt.Inverse() * ground_truth_frame
		odom_target_offset = current_target.Inverse() * odom_frame
		# 1 deg offset = 0.65 px offset; 1 m offset = 15 px offset
		combined_offset = 0.65 * math.degrees(gt_target_offset.M.GetRPY()[2]) + 15*gt_target_offset.p.y()
		expected_offset = 0.65 * math.degrees(odom_target_offset.M.GetRPY()[2]) + 15*odom_target_offset.p.y()
		# combined_offset += np.random.randn() * 2 + 0.5

		# correction_rotation = - 0.02 * (combined_offset)
		correction_rotation = - 0.02 * (combined_offset - expected_offset)
		# rotate the target offset by the rotational correction
		target_offset = tf_conversions.Frame(tf_conversions.Rotation.RotZ(correction_rotation)) * target_offset

		if target_index > 1:
			prev_target_gt = np_to_frame(targets[target_index-2])
			prev_target_offset = prev_target_gt.Inverse() * ground_truth_frame 
			next_target_offset = new_target_gt.Inverse() * ground_truth_frame 
			if next_target_offset.p.Norm() < gt_target_offset.p.Norm():
				target_offset.p *= 0.5 * (next_target_offset.p.Norm()/gt_target_offset.p.Norm())
			elif prev_target_offset.p.Norm() < gt_target_offset.p.Norm():
				target_offset.p *= 1.5 * (gt_target_offset.p.Norm()/prev_target_offset.p.Norm())

		# x_offset += np.random.randn() * 0.05# + 0.5
		# target_offset.p *= (target_offset.p.Norm() - x_offset) / target_offset.p.Norm()

		# add the offset to the current target
		target = frame_to_np(tf_conversions.Frame(target_offset.M * current_target.M, target_offset.p + current_target.p))

		target_to_navigate_to = target + np.array([0.1*math.cos(target[2]),0.1*math.sin(target[2]),0])

		actual_targets.append(target)
		update_locations.append([gt_xs[i], gt_ys[i], gt_thetas[i]])
		x_errors.append(gt_target_offset.p.x())


	vN = np.random.randn() * MAX_V / 20
	omegaN = np.random.randn() * MAX_OMEGA / 20

	odom_frame.p += tf_conversions.Vector(dt * 1.1*(v+vN) * math.cos(theta), dt * (v+vN) * math.sin(theta), 0.0)
	odom_frame.M.DoRotZ(dt * (omega+omegaN))

	ground_truth_frame.p += tf_conversions.Vector(dt * v * math.cos(gt_theta), dt * v * math.sin(gt_theta), 0.0)
	ground_truth_frame.M.DoRotZ(dt * omega)

	# display_spacing = 50
	# if i % display_spacing == 0:
	# 	line_gt.set_offsets(np.array((gt_xs[:i:display_spacing],gt_ys[:i:display_spacing])).T)
	# 	line_gt.set_UVC(np.cos(gt_thetas[:i:display_spacing]), np.sin(gt_thetas[:i:display_spacing]))
	# 	line_odom.set_offsets(np.array((xs[:i:display_spacing],ys[:i:display_spacing])).T)
	# 	line_odom.set_UVC(np.cos(thetas[:i:display_spacing]), np.sin(thetas[:i:display_spacing]))
	# 	fig.canvas.restore_region(fig_bg)
	# 	ax.draw_artist(line_targets)
	# 	ax.draw_artist(line_gt)
	# 	ax.draw_artist(line_odom)
	# 	fig.canvas.blit(ax.bbox)
	# 	fig.canvas.flush_events()



print('final pose = [%f, %f, %f]' % (xs[-1],ys[-1],thetas[-1]))
print('target = [%f, %f, %f]' % (target[0],target[1],target_theta))
print(target_index)

if len(update_locations) > targets.shape[0]:
	targets_right_len = np.tile(targets, (int(math.ceil(float(len(update_locations))/targets.shape[0])),1))[:len(update_locations),:]
else:
	targets_right_len = targets[:len(update_locations),:]
plt.plot(np.vstack((targets_right_len[:len(update_locations),0],np.array([t[0] for t in update_locations]))), 
	np.vstack((targets_right_len[:len(update_locations),1],np.array([t[1] for t in update_locations]))), 
	color="#0000ff", alpha=0.5)
plt.quiver(targets[:,0], targets[:,1], np.cos(targets[:,2]), np.sin(targets[:,2]))
# plt.quiver([t[0] for t in actual_targets], [t[1] for t in actual_targets], [math.cos(t[2]) for t in actual_targets], [math.sin(t[2]) for t in actual_targets], color="#ff0000", alpha=0.5)
plt.plot([t[0] for t in update_locations], [t[1] for t in update_locations], 'x', color="#ff0000", alpha=0.5)
display_spacing = 10
q1 = plt.quiver(gt_xs[::display_spacing], gt_ys[::display_spacing], np.cos(gt_thetas[::display_spacing]), np.sin(gt_thetas[::display_spacing]), scale=50, color='#00ff00', alpha = 0.5)
# q2 = plt.quiver(xs[::display_spacing], ys[::display_spacing], np.cos(thetas[::display_spacing]), np.sin(thetas[::display_spacing]), scale=50, color='#0000ff', alpha = 0.2)
plt.axis('equal')
# plt.legend([q1,q2],['ground truth','odometry'])
plt.legend([q1],['ground truth'])
plt.title('Navigation test - Correction - 5% Gaussian Noise [6 min]')

plt.figure()
plt.plot(x_errors)

plt.show()
