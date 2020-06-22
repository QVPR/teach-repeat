import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import collections

# 9th June 2f03a28ea03fa45fb54d060462bf99c977deb5ea (INTER_AREA size reduction)
data = collections.OrderedDict((
	('1/1 \n(115x44)', [1, 1, 1, 1, 1]),
	('1/1.5 \n(76x29)', [1, 1, 1, 1, 0]),
	('1/2 \n(57x22)', [1, 1, 1, 1, 1]),
	('1/2.5 \n(46x17)', [1, 1, 1, 1, 1]),
	('1/3 \n(38x14)', [1, 1, 1, 0, 1]),
	('1/3.5 \n(32x12)', [1, 1, 1, 1, 1]),
	('1/4 \n(28x11)', [1, 1, 1, 1, 1]),
	('1/5 \n(23x8)', [1, 1, 1, 1, 1]),
	('1/6 \n(19x7)', [1, 0, 1, 0, 1]),
	('1/8 \n(14x5)', [1, 1, 0, 1, 0]),
	('1/10 \n(11x4)', [0, 1, 1, 0, 0]),
	('1/15 \n(7x2)', [0, 0, 0, 1, 0]),
	('no \ncorrection', [0, 0, 0, 0, 0]),
))

df = pd.DataFrame(data, dtype=bool)

m = np.mean(df)
# Bernoulli stdev
s = 100*np.sqrt(m * (1-m) / (df.shape[0]-1))

matplotlib.style.use('ggplot')
# m.plot(kind='bar', yerr=s, capsize=3)
(100*m).plot(kind='bar', yerr=s, capsize=3)

plt.xticks(rotation=0)
plt.ylabel('Proportion successful tests (%)')
# plt.ylim((0,1))
plt.xlabel('Image size')
plt.title('Reliability of approach - ~15 m path [N=5]')

plt.tight_layout()

plt.show()