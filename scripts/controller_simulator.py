#!/usr/bin/env python

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
import sys

# ugly hack to import things from the nodes folder
sys.path.append(os.path.dirname(__file__) + '/../nodes')
import importlib
drive_to_pose_controller = importlib.import_module('drive_to_pose_controller')
localiser = importlib.import_module('localiser')

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

def scale_velocities(v, omega, max_v, min_omega, max_omega, stop_at_goal):
	if not stop_at_goal or abs(v) > max_v or abs(omega) > max_omega:
		# Scale to preserve rate of turn
		if omega == 0:
			v = max_v
			omega = 0
		elif v == 0:
			v = 0
			omega = math.copysign(min_omega, omega)
		else:
			turn_rate = abs(omega / v)
			turn_rate_at_max = max_omega / max_v

			if turn_rate > turn_rate_at_max:
				omega = math.copysign(max_omega, omega)
				v = max_omega / turn_rate
			else:
				omega = math.copysign(max_v * turn_rate, omega)
				v = max_v
	elif abs(omega) < min_omega and v == 0:
		omega = math.copysign(min_omega, omega)
	return v, omega

MAX_V = 0.2
MAX_OMEGA = 2.439
dt = 0.02

LOOKAHEAD_DISTANCE = localiser.LOOKAHEAD_DISTANCE_RATIO*localiser.GOAL_DISTANCE_SPACING
TURNING_TARGET_RANGE = localiser.TURNING_TARGET_RANGE_DISTANCE_RATIO*localiser.GOAL_DISTANCE_SPACING

gain_rho = 0.5
gain_alpha = 5.0
gain_beta = -3.0
gain_theta = 7.0

np.random.seed(7)

# directory = os.path.expanduser('~/miro/data/follow-long-path/')
directory = os.path.expanduser('~/miro/data/under-table/')
pose_files = get_pose_files(directory)
poses = read_pose_files(pose_files)
pose_data = get_pose_x_y_theta(poses)

goals = np.hstack([data.reshape(-1,1) for data in pose_data])
# goals = goals[:-2:1,:]
# goals = np.vstack((np.arange(0.2,10,0.2),np.zeros(49),np.zeros(49))).T
# goals = np.array([[1,0.3,math.radians(-10)]])
goal_index = 0
last_goal = np.array([0,0,0])
goal = goals[goal_index]

goal_to_navigate_to = goal + np.array([LOOKAHEAD_DISTANCE*math.cos(goal[2]), LOOKAHEAD_DISTANCE*math.sin(goal[2]),0])
turning_goal = False

actual_goals_odom = []
actual_goals_world = []
update_locations = []
x_errors = []
continuous_offsets = []
continuous_expected_offsets = []
continuous_path_offsets = []

odom = Odometry()
odom.pose.pose.position.x = 0.0
odom.pose.pose.position.y = 0.0
q = tf_conversions.Frame().M.RotZ(math.radians( 0 )).GetQuaternion()
odom.pose.pose.orientation.z = q[2]
odom.pose.pose.orientation.w = q[3]
current_frame_odom = tf_conversions.fromMsg(odom.pose.pose)

world = Pose()
world.position.x = odom.pose.pose.position.x
world.position.y = odom.pose.pose.position.y
world.orientation.z = odom.pose.pose.orientation.z
world.orientation.w = odom.pose.pose.orientation.w
current_frame_world = tf_conversions.fromMsg(world)

N = 10000
xs = []
ys = []
thetas = []
gt_xs = []
gt_ys = []
gt_thetas = []

def update_step():
	global turning_goal
	theta = current_frame_odom.M.GetRPY()[2]
	gt_theta = current_frame_world.M.GetRPY()[2]
	target_pos = (goal_to_navigate_to[0], goal_to_navigate_to[1], 0)
	target_theta = goal_to_navigate_to[2]

	d_pos = tf_conversions.Vector(*target_pos) - current_frame_odom.p
	rho, alpha, beta = drive_to_pose_controller.rho_alpha_beta(d_pos.x(), d_pos.y(), theta, target_theta)

	# v = gain_rho * rho
	# omega = gain_alpha * alpha + gain_beta * beta

	# Note: rho builds up over time if we turn for one goal, but using turning goal it never moves...
	# if rho < TURNING_TARGET_RANGE:
	# 	v = 0
	# 	omega = gain_alpha * wrapToPi(target_theta-theta)

	if rho < TURNING_TARGET_RANGE or alpha > math.pi/2 or alpha < -math.pi/2:
		# we're not in the stable region of the controller - rotate to face the goal
		# Better would be to reverse the direction of motion, but this would break along-path localisation
		v = 0
		omega = gain_theta * wrapToPi(target_theta-theta)
	else:
		# we're in the stable region of the controller
		v = gain_rho * rho
		omega = gain_alpha * alpha + gain_beta * beta

	v, omega = scale_velocities(v, omega, 0.2, 2.2, 4.8, turning_goal)

	vN = np.random.randn() * MAX_V / 10
	omegaN = np.random.randn() * MAX_OMEGA / 10

	current_frame_odom.p += tf_conversions.Vector(dt * 1*(v+vN) * math.cos(theta), dt * (v+vN) * math.sin(theta), 0.0)
	current_frame_odom.M.DoRotZ(dt * (omega+omegaN))

	current_frame_world.p += tf_conversions.Vector(dt * v * math.cos(gt_theta), dt * v * math.sin(gt_theta), 0.0)
	current_frame_world.M.DoRotZ(dt * omega)

	xs.append(current_frame_odom.p.x())
	ys.append(current_frame_odom.p.y())
	thetas.append(current_frame_odom.M.GetRPY()[2])

	gt_xs.append(current_frame_world.p.x())
	gt_ys.append(current_frame_world.p.y())
	gt_thetas.append(current_frame_world.M.GetRPY()[2])

def get_offset_px(goal, pose):
	visual_feature_range = 4#np.random.uniform(3.0, 5.0)
	visual_feature_angle = 0.02#np.random.uniform(-math.radians(10), math.radians(10))

	visual_feature_offset = tf_conversions.Frame()
	visual_feature_offset.M.DoRotZ(visual_feature_angle)
	visual_feature_offset.p.x(visual_feature_range)

	visual_feature = goal * visual_feature_offset

	offset = pose.Inverse() * visual_feature

	return -localiser.rad_to_px(visual_feature_angle - math.atan2(offset.p.y(), offset.p.x()))

def calculate_image_pose_offset(goal_index, half_search_range=None, return_all=False):
	HALF_SEARCH_RANGE = 1
	if half_search_range is None:
		half_search_range = HALF_SEARCH_RANGE

	start_range = max(0, goal_index - half_search_range)
	end_range = min(len(goals), goal_index + half_search_range + 1)

	diffs = [np_to_frame(goals[i]).Inverse() * current_frame_world for i in range(start_range, end_range)]
	dists = [frame.p.Norm() for frame in diffs]

	if goal_index >= half_search_range:
		centre_image_index = half_search_range
	else:
		centre_image_index = goal_index

	best_match = np.argmin(dists)

	path_offset_magnitude = best_match - centre_image_index
	if path_offset_magnitude > 0:
		path_offset = 0.5 ** path_offset_magnitude
	elif path_offset_magnitude < 0:
		path_offset = 1.5 ** (-path_offset_magnitude)
	else:
		path_offset = 1.0

	if return_all:
		offsets = [localiser.px_to_rad(get_offset_px(np_to_frame(goals[i]), current_frame_world)) for i in range(start_range, end_range)]
		correlations = list(np.exp(-4*np.array(dists)))
		return offsets, correlations
	else:
		px_offset = get_offset_px(np_to_frame(goals[goal_index]), current_frame_world)
		theta_offset = localiser.px_to_rad(px_offset)
		return path_offset, theta_offset

def update_goal(goal_frame, new_goal=False):
	global goal, goal_to_navigate_to, last_goal, turning_goal
	if new_goal:
		last_goal = goal
		diff = np_to_frame(goal).Inverse() * goal_frame
	else:
		diff = np_to_frame(last_goal).Inverse() * goal_frame

	goal = frame_to_np(goal_frame)

	# if goal is a turning goal, don't set a virtual waypoint ahead
	# print(diff.p.Norm())
	if diff.p.Norm() < TURNING_TARGET_RANGE:
		turning_goal = True
		goal_to_navigate_to = goal
	else:
		turning_goal = False
		goal_to_navigate_to = goal + np.array([LOOKAHEAD_DISTANCE*math.cos(goal[2]),LOOKAHEAD_DISTANCE*math.sin(goal[2]),0])

def save_data_at_goal(old_goal_frame_world, new_goal):
	actual_goals_odom.append(goal)
	update_locations.append([gt_xs[i], gt_ys[i], gt_thetas[i]])
	goal_offset_world = old_goal_frame_world.Inverse() * current_frame_world
	x_errors.append(goal_offset_world.p.x())
	goal_odom_world = frame_to_np(new_goal * current_frame_odom.Inverse() * current_frame_world)
	actual_goals_world.append(goal_odom_world)

def do_continuous_correction():
	last_goal_world = np_to_frame(goals[goal_index-1])
	next_goal_world = np_to_frame(goals[goal_index])
	last_goal_odom = np_to_frame(last_goal)
	next_goal_odom = np_to_frame(goal)

	last_goal_offset_odom = last_goal_odom.Inverse() * current_frame_odom
	next_goal_offset_odom = next_goal_odom.Inverse() * current_frame_odom
	inter_goal_offset_world = last_goal_world.Inverse() * next_goal_world
	inter_goal_distance = inter_goal_offset_world.p.Norm()
	inter_goal_offset_odom = last_goal_odom.Inverse() * next_goal_odom
	inter_goal_distance_odom = inter_goal_offset_odom.p.Norm()

	next_goal_distance = next_goal_offset_odom.p.Norm()
	last_goal_distance = last_goal_offset_odom.p.Norm()
	last_goal_to_next_goal_vector = np.array(list(inter_goal_offset_odom.p))
	last_goal_to_current_pose_vector = np.array(list(last_goal_offset_odom.p))
	next_goal_angle = next_goal_offset_odom.M.GetRPY()[2]
	last_goal_angle = last_goal_offset_odom.M.GetRPY()[2]

	# if it's a distance goal, use distance; if it's a rotation goal, use angle
	if inter_goal_distance < 0.1:
		u = last_goal_angle / (last_goal_angle + next_goal_angle)
	else:
		u = np.sum(last_goal_to_next_goal_vector * last_goal_to_current_pose_vector) / np.sum(last_goal_to_next_goal_vector**2)

	SR = 1
	offsets, correlations = calculate_image_pose_offset(goal_index, 1+SR, return_all=True)
	if goal_index > SR:
		last_offset = offsets[SR]
		next_offset = offsets[SR+1]
	else:
		last_offset = offsets[-SR-3]
		next_offset = offsets[-SR-2]
	expected_last_offset = localiser.px_to_rad(localiser.get_expected_px_offset(last_goal_offset_odom))
	expected_next_offset = localiser.px_to_rad(localiser.get_expected_px_offset(next_goal_offset_odom))

	offset = (1-u) * (last_offset) + u * (next_offset)
	expected_offset = (1-u) * expected_last_offset + u * expected_next_offset

	K = 0.01
	correction_rad = K * (offset)#- expected_offset)

	# if math.copysign(1, correction_rad) != math.copysign(1, offset):
	# 	correction_rad = 0.0

	if goal_index > SR and goal_index < len(goals)-SR:
		corr = np.array(correlations[:2*(1+SR)])
		corr -= 0.1
		corr[corr < 0] = 0.0
		s = corr.sum()
		if s > 0:
			corr /= s
		w = corr * np.arange(-0.5-SR,0.6+SR,1)
		pos = w.sum()
		path_error = pos - (u - 0.5)

		K2 = 0.01
		path_correction = (next_goal_distance - K2 * path_error * localiser.GOAL_DISTANCE_SPACING) / next_goal_distance
	else:
		path_correction = 1.0

	goal_offset = localiser.get_corrected_goal_offset(current_frame_odom, next_goal_odom, correction_rad, path_correction)
	new_goal = current_frame_odom * goal_offset

	update_goal(new_goal, False)

	continuous_offsets.append(offset)
	continuous_expected_offsets.append((1-u) * expected_last_offset + u * expected_next_offset)
	continuous_path_offsets.append(expected_offset)
	# continuous_path_offsets.append(path_correction)

for i in range(N):
	update_step()

	current_goal_frame_odom = np_to_frame(goal)
	old_goal_frame_world = np_to_frame(goals[goal_index])

	delta_frame = current_frame_odom.Inverse() * np_to_frame(goal_to_navigate_to)

	if goal_index > 0:
		do_continuous_correction()

	if localiser.delta_frame_in_bounds(delta_frame):
		old_goal_index = goal_index

		goal_index += 1
		if goal_index == len(goals):
			break

		new_goal_frame_world = np_to_frame(goals[goal_index])
		image_path_offset, image_rad_offset = calculate_image_pose_offset(old_goal_index)

		known_goal_offset = current_goal_frame_odom.Inverse() * current_frame_odom
		expected_theta_offset = localiser.px_to_rad(localiser.get_expected_px_offset(known_goal_offset))
		correction_rad = 0#image_rad_offset - expected_theta_offset
		path_correction = 1.0#image_path_offset

		goal_offset = localiser.get_corrected_goal_offset(old_goal_frame_world, new_goal_frame_world, correction_rad, path_correction)
		new_goal = current_goal_frame_odom * goal_offset

		update_goal(new_goal, True)

		save_data_at_goal(old_goal_frame_world, new_goal)


if len(update_locations) > goals.shape[0]:
	goals_right_length = np.tile(goals, (int(math.ceil(float(len(update_locations))/goals.shape[0])),1))[:len(update_locations),:]
else:
	goals_right_length = goals[:len(update_locations),:]
actual_goals_world = np.array(actual_goals_world)
actual_goals_odom = np.array(actual_goals_odom)
update_locations = np.array(update_locations)
# print('ERROR = %f' % math.sqrt(np.mean((goals_right_length[:,:2] - actual_goals_world[:,:2])**2)))
print('ERROR = %f' % math.sqrt(np.mean((goals_right_length[:,:2] - update_locations[:,:2])**2)))

display_spacing = 10

# plt.plot(continuous_offsets)
# plt.figure()
# plt.plot(continuous_expected_offsets)
# plt.figure()
# plt.plot([off - exp for off,exp in zip(continuous_offsets, continuous_expected_offsets)])
# plt.figure()
# plt.quiver(actual_goals_odom[:,0], actual_goals_odom[:,1], np.cos(actual_goals_odom[:,2]), np.sin(actual_goals_odom[:,2]))
# plt.quiver(xs[::display_spacing], ys[::display_spacing], np.cos(thetas[::display_spacing]), np.sin(thetas[::display_spacing]), scale=50, color='#00ff00', alpha = 0.5)
# plt.plot(continuous_offsets)
# plt.plot(continuous_path_offsets)
plt.figure()
plt.plot(np.vstack((goals_right_length[:len(update_locations),0],np.array([t[0] for t in update_locations]))), 
	np.vstack((goals_right_length[:len(update_locations),1],np.array([t[1] for t in update_locations]))), 
	color="#0000ff", alpha=0.5)
plt.quiver(goals[:,0], goals[:,1], np.cos(goals[:,2]), np.sin(goals[:,2]))
# plt.quiver([t[0] for t in actual_targets], [t[1] for t in actual_targets], [math.cos(t[2]) for t in actual_targets], [math.sin(t[2]) for t in actual_targets], color="#ff0000", alpha=0.5)
# plt.plot([t[0] for t in update_locations], [t[1] for t in update_locations], 'x', color="#ff0000", alpha=0.5)
q1 = plt.quiver(gt_xs[::display_spacing], gt_ys[::display_spacing], np.cos(gt_thetas[::display_spacing]), np.sin(gt_thetas[::display_spacing]), scale=50, color='#00ff00', alpha = 0.5)
# plt.quiver(target_errors_world[:,0], target_errors_world[:,1], np.cos(target_errors_world[:,2]), np.sin(target_errors_world[:,2]), color='#ff0000', alpha = 0.5)
# q2 = plt.quiver(xs[::display_spacing], ys[::display_spacing], np.cos(thetas[::display_spacing]), np.sin(thetas[::display_spacing]), scale=50, color='#0000ff', alpha = 0.2)
plt.axis('equal')
# plt.legend([q1,q2],['ground truth','odometry'])
plt.legend([q1],['ground truth'])
plt.title('Navigation test - Correction - 5% Gaussian Noise [6 min]')


plt.show()
