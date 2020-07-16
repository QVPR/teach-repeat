import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import collections
import copy

# deadspots = 175.2 (full fov) / res / 2 * discretisation
# eg. 115x44 disc. 5 -> 175.2/115/2*5 = 3.81 deg
# 2nd July (93a92648a5e774c97e3368e3306782382f626b6d) - SR=1, rho=0.1, theta=5 deg
data = {
	'115x44':{
		'3.81':  [1, 1, 1, 0, 1, 1, 1, 1, 1, 1],
		'7.62':  [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
		'11.43': [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
	},
	'23x9':{
		'3.81':  [0, 1, 1, 1, 1, 0, 1, 1, 1, 1],
		'7.62':  [0, 0, 1, 0, 1, 1, 0, 0, 0, 1],
		'11.43': [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
	},
}

mean = {}
stdev = np.zeros((len(data.keys()),max([len(v) for k,v in data.items()])))
for i, (k,v) in enumerate(data.items()):
	mean[k] = {}
	for j, (k2, v2) in enumerate(v.items()):
		mean[k][k2] = np.mean(v2)
		stdev[i,j] = mean[k][k2] * (1-mean[k][k2]) / (len(v2)-1)

df = pd.DataFrame(mean)

matplotlib.style.use('ggplot')
order = np.argsort([float(x) for x in df.index])
stdev = stdev[::-1,:]
(df*100).iloc[order].plot(kind='bar', yerr=100*stdev, capsize=3)

plt.xticks(rotation=0)
plt.ylabel('Proportion successful tests (%)')
plt.ylim((0,100))
plt.xlabel('"Dead spot", minimum detected orientation offset $(\circ)$')
plt.title('Artificially reduced offset resolution for larger images\nproduces performance similar to smaller images\n[Discrete correction, N=10]')
plt.tight_layout()

# deadspots = 175.2 (full fov) / res / 2 * discretisation
# eg. 115x44 disc. 5 -> 175.2/115/2*5 = 3.81 deg
# 2nd July (93a92648a5e774c97e3368e3306782382f626b6d) - SR=1, rho=0.1, theta=5 deg
data = {
	'discrete 23x9':{
		'3.81':  [0, 1, 1, 1, 1, 0, 1, 1, 1, 1],
		'7.62':  [0, 0, 1, 0, 1, 1, 0, 0, 0, 1],
		'11.43': [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
	},
	'continuous 23x9':{
		'3.81':  [0, 1, 1, 1, 1],
		'7.62':  [1, 1, 1, 1, 1],
		'11.43': [1, 1, 0, 1, 1],
		'19.04': [0, 1, 1, 0, 0],
		'26.66': [1, 0, 0, 1, 0],
		'38.09': [0, 0, 0, 0, 0],
	},
}

mean = {}
# stdev = np.zeros((len(data.keys()),max([len(v) for k,v in data.items()])))
for i, (k,v) in enumerate(data.items()):
	mean[k] = {}
	for j, (k2, v2) in enumerate(v.items()):
		mean[k][k2] = np.mean(v2)
		# stdev[i,j] = mean[k][k2] * (1-mean[k][k2]) / (len(v2)-1)

df = pd.DataFrame(mean)

order = np.argsort([float(x) for x in df.index])
stdev = stdev[::-1,:]
(df*100).iloc[order].plot(kind='bar', capsize=3)

plt.xticks(rotation=0)
plt.ylabel('Proportion successful tests (%)')
plt.ylim((0,100))
plt.xlabel('"Dead spot", minimum detected orientation offset $(\circ)$')
plt.title('Continuous correction works at a lower offset resolution')
plt.tight_layout()

data = collections.OrderedDict((
	('normal', 0.4),
	('middle', 0.8),
	('depth', 0.6),
	('both', 0.6),
	('no correction', 0.2),
))

stdev = np.array(data.values())
stdev *= (1 - stdev) / 4.

df = pd.DataFrame(data.values(), index=data.keys())

(df*100).plot(kind='bar', yerr=100*stdev, capsize=3, legend=False)

plt.xticks(rotation=0)
plt.ylabel('Proportion successful tests (%)')
plt.ylim((0,100))
plt.xlabel('Reference image weighting type')
plt.title('Weighting reference images can improve performance')
plt.tight_layout()

plt.show()