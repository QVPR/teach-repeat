#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import tf_conversions

import math

import numpy as np
import matplotlib.pyplot as plt

MAX_V = 0.4
MAX_OMEGA = 2.439

def wrapToPi(x):
	return ((x + math.pi) % (2*math.pi)) - math.pi

def rho_alpha_beta(dx, dy, theta, goal_theta):
	rho = math.sqrt(dx*dx + dy*dy)
	alpha = wrapToPi(math.atan2(dy, dx) - theta)
	beta = wrapToPi(-theta - alpha + goal_theta)
	return rho, alpha, beta

def scale_velocities(v, omega, force):
	if force or abs(v) > MAX_V or abs(omega) > MAX_OMEGA:
		# Scale to preserve rate of turn
		if omega == 0:
			v = MAX_V
			omega = 0
		elif v == 0:
			v = 0
			omega = MAX_OMEGA * omega / abs(omega)
		else:
			turn_rate = abs(omega / v)
			turn_rate_at_max = MAX_OMEGA / MAX_V

			if turn_rate > turn_rate_at_max:
				omega = MAX_OMEGA * omega / abs(omega)
				v = MAX_OMEGA / turn_rate * v / abs(v)
			else:
				omega = MAX_V * turn_rate * omega / abs(omega)
				v = MAX_V * v / abs(v)
	return v, omega

limit_rho = 0.2
limit_theta = math.radians(15)

K_rho = 1
K_alpha = 5
K_beta = -3

# constant angular velocity, omega
omega = 2
v = 0.1

t_omega_lim = limit_theta / omega
t_rho_lim = 1 - (limit_rho * omega) / (v * math.sqrt(2))
if abs(t_rho_lim) > 1:
	t_rho_lim = np.inf
else:
	t_rho_lim = math.acos(t_rho_lim)

if t_omega_lim < t_rho_lim:
	t_end = t_omega_lim
	print('omega hits limit at t = %f' % t_omega_lim)
else:
	t_end = t_rho_lim
	print('rho hits limit at t = %f' % t_rho_lim)

x_end = v/omega * np.sin(omega*t_end)
y_end = v/omega * (1 - np.cos(omega*t_end))
theta_end = omega * t_end

target = np.array([x_end, y_end, theta_end])
N = 50
dt = 0.02
pos = np.zeros((N,3))
for i in range(1, N):
	diff = target - pos[i-1,:]
	theta = pos[i-1,2]
	rho, alpha, beta = rho_alpha_beta(diff[0], diff[1], theta, theta_end)
	control_v = K_rho * rho
	control_omega = K_alpha*alpha + K_beta*beta
	control_v, control_omega = scale_velocities(control_v, control_omega, True)
	pos[i,:] = pos[i-1,:] + dt*np.array([control_v*math.cos(theta),control_v*math.sin(theta),control_omega]) + 0.0001*np.random.randn(3)


t = np.arange(0, t_end, 0.02)

plt.plot(v/omega * np.sin(omega*t), v/omega * (1 - np.cos(omega*t)))
plt.quiver(x_end, y_end, math.cos(theta_end), math.sin(theta_end))
plt.plot(pos[:,0],pos[:,1],'--')
plt.axis('equal')

ranges = np.arange(0,limit_rho,limit_rho/10.)
psis = np.arange(-limit_theta, limit_theta+.01, limit_theta/10.)
thetas = np.arange(-limit_theta, limit_theta+.01, limit_theta/10.)
values = np.zeros((ranges.size,psis.size, thetas.size))

for i,r in enumerate(ranges):
	for j,psi in enumerate(psis):
		for k,theta in enumerate(thetas):
			rho, alpha, beta = rho_alpha_beta(r*math.cos(psi), r*math.sin(psi), 0, theta)
			values[i,j,k] = K_alpha*alpha + K_beta*beta

plt.figure()
plt.imshow(values[:,:,15])
plt.colorbar()

plt.show()