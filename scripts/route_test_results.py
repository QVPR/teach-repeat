import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt

labels = ['Initial y offset', 'Final x error', 'Final y error']
data = [
	[0, 0, 0.25],
	[0, -0.05, 0.15],
	[0, -0.05, 0.10],
	[0.5, -.1, .2],
	[0.5, -.1, .2],
	[0.5, -.1, .25],
	[-0.5, -.05, -.05],
	[-0.5, -.1, -.1],
	[-0.5, np.nan, np.nan],
	[1.5, -.45, .3],
	[1.5, -.5, .7],
	[1.5, -.45, .5]
]

df = pd.DataFrame(data, columns=labels, dtype=float)

initial_offsets = np.unique(df['Initial y offset'])
results = {}

for offset in initial_offsets:
	data = df[df['Initial y offset'] == offset]
	x = data['Final x error']
	y = data['Final y error']
	norm = np.sqrt(x*x + y*y)

	results['offset %.1f' % offset] = {
		'x_mean': np.mean(x),
		'y_mean': np.mean(y),
		'norm_mean': np.mean(norm),
		'x_std': np.std(x),
		'y_std': np.std(y),
		'norm_std': np.std(norm),
	}

print(pd.DataFrame(results, dtype=float).loc['x_mean'])

matplotlib.style.use('ggplot')
plt.scatter(df['Final x error'], df['Final y error'], c=df['Initial y offset'])
plt.legend()

plt.show()