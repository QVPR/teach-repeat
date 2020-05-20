import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt

data = {
	0.7: [1, 1, 1, 1, 1, 0, 0, 1, 0, 0],
	0.8: [1, 1, 1, 0, 1, 1, 1, 0, 0, 1],
	0.9: [1, 1, 1, 1, 1, 1, 0, 0, 1, 1],
	1.0: [1, 1, 1, 1, 1, 1, 0, 1, 1, 1],
	1.1: [1, 1, 0, 1, 1, 1, 0, 0, 1, 1],
	1.2: [1, 0, 0, 1, 1, 1, 0, 0, 1, 0],
	1.3: [0, 1, 1, 0, 0, 0, 0, 0, 0, 0]
}

df = pd.DataFrame(data, dtype=bool)

m = np.mean(df)
# Bernoulli stdev
s = np.sqrt(m * (1-m) / (df.shape[0]-1))

matplotlib.style.use('ggplot')
m.plot(kind='bar', yerr=s, capsize=3)

plt.xticks(rotation=0)
plt.ylabel('Proportion successful tests (%)')
# plt.ylim((0,1))
plt.xlabel('Perceived odometry multiplier')
plt.title('Reliability of approach - 10 tests along ~15 m path')

plt.show()