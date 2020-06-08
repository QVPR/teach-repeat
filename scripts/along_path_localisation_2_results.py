import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import collections

# 20th May (25555aa8f2554362fdc059d85a6a6d07ab5ff2e1) - SR=1, rho=0.1, theta=5 deg
data = {
	0.7: [1, 1, 1, 1, 1, 0, 0, 1, 0, 0],
	0.8: [1, 1, 1, 0, 1, 1, 1, 0, 0, 1],
	0.9: [1, 1, 1, 1, 1, 1, 0, 0, 1, 1],
	1.0: [1, 1, 1, 1, 1, 1, 0, 1, 1, 1],
	1.1: [1, 1, 0, 1, 1, 1, 0, 0, 1, 1],
	1.2: [1, 0, 0, 1, 1, 1, 0, 0, 1, 0],
	1.3: [0, 1, 1, 0, 0, 0, 0, 0, 0, 0]
}
# 9th June (036597cd43003130d2ba8866314418717d6edbaa) - SR=1, rho=0.1, theta=5 deg, K=K2=0.01
data2 = {
	0.7: [0, 1, 0, 1, 0],
	0.8: [1, 1, 0, 1, 1],
	0.9: [1, 1, 1, 1, 1],
	1.0: [1, 1, 1, 1, 1],
	1.1: [1, 1, 1, 1, 1],
	1.2: [1, 1, 1, 1, 1],
	1.3: [1, 1, 1, 0, 0]
}

df = pd.DataFrame(data, dtype=bool)
df2 = pd.DataFrame(data2, dtype=bool)

m = np.mean(df)
m2 = np.mean(df2)
# Bernoulli stdev
s = np.sqrt(m * (1-m) / (df.shape[0]-1))
s2 = np.sqrt(m2 * (1-m2) / (df2.shape[0]-1))

data = 100*pd.DataFrame(collections.OrderedDict((('discrete correction (N=10)',m),('continuous correction (N=5)',m2))), index=m.index)
std = [100*s, 100*s2]

matplotlib.style.use('ggplot')
# m.plot(kind='bar', yerr=s, capsize=3)
data.plot(kind='bar', yerr=std, capsize=3)

plt.xticks(rotation=0)
plt.ylabel('Proportion successful tests (%)')
# plt.ylim((0,1))
plt.xlabel('Perceived odometry multiplier')
plt.title('Reliability of approach - ~15 m path')

plt.show()