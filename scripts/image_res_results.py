#%%


import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import collections
import seaborn as sns; sns.set(); sns.set_style("whitegrid"); sns.set_context("notebook", font_scale=1.0)
from matplotlib import rc
rc('font',**{'family':'sans-serif','sans-serif':['Computer Modern Sans serif']})
rc('text', usetex=True)
matplotlib.rcParams['xtick.major.pad'] = -3


# 9th June 2f03a28ea03fa45fb54d060462bf99c977deb5ea (INTER_AREA size reduction)
# both gains were set to 0.01
# Note: corrections weren't properly scaled with the image size, 11 px image was 1/10th gain of normal
data = collections.OrderedDict((
	('1/1 (115x44)', [1, 1, 1, 1, 1]),
	('1/1.5 (76x29)', [1, 1, 1, 1, 0]),
	('1/2 (57x22)', [1, 1, 1, 1, 1]),
	('1/2.5 (46x17)', [1, 1, 1, 1, 1]),
	('1/3 (38x14)', [1, 1, 1, 0, 1]),
	('1/3.5 (32x12)', [1, 1, 1, 1, 1]),
	('1/4 (28x11)', [1, 1, 1, 1, 1]),
	('1/5 (23x8)', [1, 1, 1, 1, 1]),
	('1/6 (19x7)', [1, 0, 1, 0, 1]),
	('1/8 (14x5)', [1, 1, 0, 1, 0]),
	('1/10 (11x4)', [0, 1, 1, 0, 0]),
	('1/15 (7x2)', [0, 0, 0, 1, 0]),
	('no correction', [0.02, 0.02, 0.02, 0.02, 0.02]),
))

df = pd.DataFrame(data, dtype=float)

m = np.mean(df)
# Bernoulli stdev
# s = 100*np.sqrt(m * (1-m) / (df.shape[0]-1))

plt.figure(figsize=(4, 1.6))

# matplotlib.style.use('ggplot')
# m.plot(kind='bar', yerr=s, capsize=3)
(100*m).plot(kind='bar', edgecolor=None, linewidth=0)

plt.xticks(rotation=45, ha="right", rotation_mode="anchor")
plt.ylabel('Proportion of\nsuccessful tests (\%)')
plt.yticks(np.arange(0, 101, step=20))

# plt.ylim((0,1))
plt.xlabel('Image size')
plt.title(r'Reliability of approach along ${\approx}$15m path [N=5]')
plt.gca().xaxis.grid(False)

# plt.tight_layout()
plt.savefig('image-resolution-test.pdf', bbox_inches='tight')
plt.show()

# %%

# %%
