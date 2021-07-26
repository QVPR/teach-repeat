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
from mpl_toolkits.mplot3d import Axes3D
import scipy.ndimage

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

def shift_1d_vector(vector, distance):
	if abs(distance) >= 1:
		integer_component = int(distance)
		vector = shift_1d_vector_integer(vector, integer_component)
		distance -= integer_component
	
	if distance > 0:
		return (1 - distance) * vector + distance * shift_1d_vector_integer(vector, 1)
	else:
		return (1 - distance) * vector + -distance * shift_1d_vector_integer(vector, -1)

def shift_1d_vector_integer(vector, distance):
	if distance > 0:
		vector = np.hstack((np.zeros(distance), vector[:-distance]))
	else:
		distance = -distance
		vector = np.hstack((vector[distance:], np.zeros(distance)))
	return vector

MAX_V = 0.2
MAX_OMEGA = 2.439
dt = 0.02

TARGET_SPACING = localiser.GOAL_DISTANCE_SPACING
LOOKAHEAD_DISTANCE = localiser.LOOKAHEAD_DISTANCE_RATIO * TARGET_SPACING
TURNING_TARGET_RANGE = localiser.TURNING_TARGET_RANGE_DISTANCE_RATIO * TARGET_SPACING

gain_rho = 0.3
gain_alpha = 5.0
gain_beta = -3.0

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

# Markov state
STATES_PER_GOAL = 5
STATE_DISTANCE = TARGET_SPACING / STATES_PER_GOAL
ODOM_VARIANCE_PER_METRE = 0.01
STATES_UPDATED_AT_ONCE = 71
STATE_UPDATE_HALF_SEARCH_RANGE = (STATES_UPDATED_AT_ONCE-1) / STATES_PER_GOAL / 2
STATE_OBSERVATION_ADDITIVE_NOISE = 5. / STATES_UPDATED_AT_ONCE
d = np.zeros(STATES_PER_GOAL * (len(goals) - 1) + 1)
d_x = np.arange(0, len(goals)-1+1./STATES_PER_GOAL, 1./STATES_PER_GOAL)
d[0]  = 1 # initialise at the start
dist_moved = 0

ds = [d]

# EKF
theta_delta = 0
theta_delta_variance = 0.1
WHEEL_VARIANCE_PER_METRE = 0.0003
WHEELBASE = 0.164
R_ALPHA = 0.01

theta_deltas = [theta_delta]
theta_delta_variances = [theta_delta_variance]

N = 10000
xs = []
ys = []
thetas = []
gt_xs = []
gt_ys = []
gt_thetas = []
turning_goals = []

def along_path_prediction(delta_d):
	global d, dist_moved
	# This is a bit horrible, but we need a hack to get this to work
	# from https://github.com/AtsushiSakai/PythonRobotics/blob/a3808bca79e22a7fc7a131d6f357fca2f30c6d75/Localization/histogram_filter/histogram_filter.py
	# They say "Prediction update is only performed when the distance traveled is larger than the distance between states."
	# so this seems reasonable
	dist_moved += delta_d
	if dist_moved >= STATE_DISTANCE:
		states_moved = int(dist_moved / STATE_DISTANCE)
		d = shift_1d_vector(d, states_moved)
		odom_var = states_moved * ODOM_VARIANCE_PER_METRE / STATE_DISTANCE
		d = scipy.ndimage.gaussian_filter1d(d, math.sqrt(odom_var))

		dist_moved = dist_moved % STATE_DISTANCE
		ds.append(d)

def along_path_observation():
	global d
	current_goal = int(np.argmax(d) / STATES_PER_GOAL)
	offsets, correlations = calculate_image_pose_offset(current_goal, STATE_UPDATE_HALF_SEARCH_RANGE, return_all=True)
	
	observation_probabilities = np.zeros_like(d)

	correlations = np.array(correlations).reshape(-1,1)
	interpolated_correlations = np.hstack((
		np.hstack((correlations[:-1],correlations[1:])).dot(
			np.vstack((np.linspace(1,0,STATES_PER_GOAL+1)[:-1],np.linspace(0,1,STATES_PER_GOAL+1)[:-1]))).flatten()
		,correlations[-1]))

	interpolated_correlations /= interpolated_correlations.sum()
	interpolated_correlations += STATE_OBSERVATION_ADDITIVE_NOISE
	interpolated_correlations /= interpolated_correlations.sum()

	if current_goal > STATE_UPDATE_HALF_SEARCH_RANGE:
		observation_probabilities[(current_goal-STATE_UPDATE_HALF_SEARCH_RANGE) * STATES_PER_GOAL:1+(current_goal+STATE_UPDATE_HALF_SEARCH_RANGE) * STATES_PER_GOAL] = interpolated_correlations
	else:
		observation_probabilities[:len(interpolated_correlations)] = interpolated_correlations

	d *= observation_probabilities
	d /= d.sum()

def orientation_prediction(delta_theta_reference, delta_theta_measurement, dL, dR):
	global theta_delta, theta_delta_variance
	theta_delta += delta_theta_measurement - delta_theta_reference
	theta_delta_variance += WHEEL_VARIANCE_PER_METRE * (abs(dL) + abs(dR)) / WHEELBASE**2

	theta_deltas.append(theta_delta)
	theta_delta_variances.append(theta_delta_variance)

def orientation_observation():
	global theta_delta, theta_delta_variance
	current_goal = int(np.argmax(d) / STATES_PER_GOAL)
	u = float(np.argmax(d)) / STATES_PER_GOAL - float(current_goal)
	offsets, correlations = calculate_image_pose_offset(current_goal, 1, return_all=True)

	if current_goal == 0:
		theta_delta_A = -offsets[0]
		theta_delta_B = -offsets[1]
	elif current_goal+1 < len(goals):
		theta_delta_A = -offsets[1]
		theta_delta_B = -offsets[2]
	else:
		return

	if u != 1:
		S_A = 1./(1-u) * theta_delta_variance + R_ALPHA
		W_A = theta_delta_variance * 1./S_A
		V_A = theta_delta_A - theta_delta
		expectation_likelihood_A = V_A * 1./(S_A / (1-u)) * V_A

		if expectation_likelihood_A < 6.3:
			theta_delta += W_A*V_A
			theta_delta_variance -= W_A*theta_delta_variance

	if u != 0:
		S_B = 1./u * theta_delta_variance + R_ALPHA
		W_B = theta_delta_variance * 1./S_B
		V_B = theta_delta_B - theta_delta
		expectation_likelihood_B = V_B * 1./(S_B / (1-u)) * V_B

		if expectation_likelihood_B < 6.3:
			theta_delta += W_B*V_B
			theta_delta_variance -= W_B*theta_delta_variance

def get_d_position():
	return np.sum(d * d_x)

def update_step():
	global turning_goal, theta_delta
	pos = get_d_position()
	goal_to_navigate_to = goals[int(math.ceil(pos))]

	theta = current_frame_odom.M.GetRPY()[2]
	gt_theta = current_frame_world.M.GetRPY()[2]
	target_pos = (goal_to_navigate_to[0], goal_to_navigate_to[1], 0)
	target_theta = goal_to_navigate_to[2]

	d_pos = tf_conversions.Vector(*target_pos) - current_frame_odom.p
	rho, alpha, beta = drive_to_pose_controller.rho_alpha_beta(d_pos.x(), d_pos.y(), theta, target_theta)

	# v = gain_rho * rho
	# omega = gain_alpha * alpha + gain_beta * beta
	v = 0.10
	omega = -1.5 * theta_delta

	turn_rotation = math.degrees(wrapToPi(goals[int(math.ceil(pos))][2] - goals[int(math.ceil(pos))-1][2]))
	turn_distance = math.sqrt(np.sum((goals[int(math.ceil(pos))][:2] - goals[int(math.ceil(pos))-1][:2])**2))
	turning_goal = abs(turn_rotation / turn_distance) > 150 # deg / m
	turning_goals.append(turning_goal)

	if turning_goal:
		# todo: do I need to disable the correction on sharp turns?
		v = 0.05
		# omega *= 10
		omega = 1 * wrapToPi(target_theta-theta)
		pass

	# Note: rho builds up over time if we turn for one goal, but using turning goal it never moves...
	# if rho < TURNING_TARGET_RANGE:
	# 	v = 0
	# 	omega = gain_alpha * wrapToPi(target_theta-theta)

	# v, omega = drive_to_pose_controller.scale_velocities(v, omega, False)

	v_encoder = v #+ np.random.randn() * MAX_V / 10
	omega_encoder = omega + np.random.randn() * MAX_OMEGA / 10

	current_frame_odom.p += tf_conversions.Vector(dt * 1*v_encoder * math.cos(theta), dt * v_encoder * math.sin(theta), 0.0)
	current_frame_odom.M.DoRotZ(dt * omega_encoder)

	current_frame_world.p += tf_conversions.Vector(dt * v * math.cos(gt_theta), dt * v * math.sin(gt_theta), 0.0)
	current_frame_world.M.DoRotZ(dt * omega)

	# todo: make it so that this still works during sharp turns
	along_path_prediction(dt * v_encoder)

	dR = (WHEELBASE*dt*omega_encoder + 2*dt*v_encoder) / 2.
	dL = 2.*dt*v - dR
	orientation_prediction(dt * omega, dt * omega_encoder, dL, dR)

	xs.append(current_frame_odom.p.x())
	ys.append(current_frame_odom.p.y())
	thetas.append(current_frame_odom.M.GetRPY()[2])

	gt_xs.append(current_frame_world.p.x())
	gt_ys.append(current_frame_world.p.y())
	gt_thetas.append(current_frame_world.M.GetRPY()[2])

def get_offset_px(goal, pose):
	visual_feature_range = 1#np.random.uniform(3.0, 5.0)
	visual_feature_angle = 0#np.random.uniform(-math.radians(10), math.radians(10))

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
	next_goal_angle = next_goal_offset_odom.M.GetRPY()[2]
	last_goal_angle = last_goal_offset_odom.M.GetRPY()[2]

	# if it's a distance goal, use distance; if it's a rotation goal, use angle
	if inter_goal_distance < 0.1:
		u = last_goal_angle / (last_goal_angle + next_goal_angle)
	else:
		u = last_goal_distance / (last_goal_distance + next_goal_distance)

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

	K = 0#.2
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

		K2 = 0#.5
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

	pos = get_d_position()
	turn_rotation = math.degrees(wrapToPi(goals[int(pos)][2] - goals[int(pos)-1][2]))
	turn_distance = math.sqrt(np.sum((goals[int(pos)][:2] - goals[int(pos)-1][:2])**2))
	turning_goal = abs(turn_rotation / turn_distance) > 150 # deg / m
	if not turning_goal:
		along_path_observation()
		orientation_observation()
		
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
xs_turning = [x for x, turn in zip(gt_xs,turning_goals) if turn]
ys_turning = [y for y, turn in zip(gt_ys,turning_goals) if turn]
thetas_turning = [theta for theta, turn in zip(gt_thetas,turning_goals) if turn]
xs_notturning = [x for x, turn in zip(gt_xs,turning_goals) if not turn]
ys_notturning = [y for y, turn in zip(gt_ys,turning_goals) if not turn]
thetas_notturning = [theta for theta, turn in zip(gt_thetas,turning_goals) if not turn]
plt.quiver(xs_turning[::display_spacing], ys_turning[::display_spacing], np.cos(thetas_turning[::display_spacing]), np.sin(thetas_turning[::display_spacing]), scale=50, color='#00ff00', alpha = 0.5)
plt.quiver(xs_notturning[::display_spacing], ys_notturning[::display_spacing], np.cos(thetas_notturning[::display_spacing]), np.sin(thetas_notturning[::display_spacing]), scale=50, color='#0000ff', alpha = 0.5)
# q1 = plt.quiver(gt_xs[not turning_goal][::display_spacing], gt_ys[not turning_goal][::display_spacing], np.cos(gt_thetas[not turning_goal][::display_spacing]), np.sin(gt_thetas[not turning_goal][::display_spacing]), scale=50, color='#00ff00', alpha = 0.5)


# plt.quiver(target_errors_world[:,0], target_errors_world[:,1], np.cos(target_errors_world[:,2]), np.sin(target_errors_world[:,2]), color='#ff0000', alpha = 0.5)
# q2 = plt.quiver(xs[::display_spacing], ys[::display_spacing], np.cos(thetas[::display_spacing]), np.sin(thetas[::display_spacing]), scale=50, color='#0000ff', alpha = 0.2)
plt.axis('equal')
# plt.legend([q1,q2],['ground truth','odometry'])
# plt.legend([q1],['ground truth'])
plt.title('Navigation test - Correction - 5% Gaussian Noise [6 min]')

ds_2d = np.array(ds)
x = np.arange(0,len(goals)-1+1./STATES_PER_GOAL, 1./STATES_PER_GOAL)

plt.figure()
plt.imshow(ds_2d)
plt.figure()
plt.plot(theta_deltas)
plt.figure()
plt.plot(np.sum(ds_2d * d_x, axis=1))

# fig = plt.figure()
# ax = fig.gca(projection='3d')
# x = np.arange(0, len(goals)-1 + 1./STATES_PER_GOAL, 1./STATES_PER_GOAL)
# y = np.arange(0, ds_2d.shape[0])
# X, Y = np.meshgrid(x, y)
# ax.plot_surface(X, Y, ds_2d)

plt.show()
