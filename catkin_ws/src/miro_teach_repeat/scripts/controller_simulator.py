
import tf_conversions
from nav_msgs.msg import Odometry
import math
import numpy as np
import matplotlib.pyplot as plt

def wrapToPi(x):
	return ((x + math.pi) % (2*math.pi)) - math.pi

MAX_V = 0.4
MAX_OMEGA = 2.439
dt = 0.02

gain_rho = 0.3
gain_alpha = 5.0
gain_beta = -3.0

targets = np.array([
	[2.4, -1.8, -math.pi/2],
	[3.6, -1.8, math.pi/2],
	[2.4, 0, math.pi]
])
target_index = 0

odom = Odometry()
odom.pose.pose.position.x = 0.0
odom.pose.pose.position.y = 0.0
q = tf_conversions.Frame().M.RotZ(math.radians( -0 )).GetQuaternion()
odom.pose.pose.orientation.z = q[2]
odom.pose.pose.orientation.w = q[3]

N = 1700
xs = np.zeros(N)
ys = np.zeros(N)
thetas = np.zeros(N)

for i in range(N):
	current_frame = tf_conversions.fromMsg(odom.pose.pose)
	theta = current_frame.M.GetRPY()[2]

	xs[i] = current_frame.p.x()
	ys[i] = current_frame.p.y()
	thetas[i] = theta

	target = targets[target_index]
	target_pos = (target[0], target[1], 0)
	target_theta = target[2]

	d_pos = tf_conversions.Vector(*target_pos) - current_frame.p

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

	if rho < 0.1:
		target_index += 1
		if target_index == len(targets):
			break

	odom.pose.pose.position.x += dt * v * math.cos(theta)
	odom.pose.pose.position.y += dt * v * math.sin(theta)
	current_frame.M.DoRotZ(dt * omega )
	odom.pose.pose.orientation.z = current_frame.M.GetQuaternion()[2]
	odom.pose.pose.orientation.w = current_frame.M.GetQuaternion()[3]

print('final pose = [%f, %f, %f]' % (xs[-1],ys[-1],thetas[-1]))
print('target = [%f, %f, %f]' % (target[0],target[1],target_theta))

plt.quiver(xs[::10], ys[::10], np.cos(thetas[::10]), np.sin(thetas[::10]), scale=50, color='#00ff00')
plt.quiver(targets[:,0], targets[:,1], np.cos(targets[:,2]), np.sin(targets[:,2]))
plt.axis('equal')

plt.show()