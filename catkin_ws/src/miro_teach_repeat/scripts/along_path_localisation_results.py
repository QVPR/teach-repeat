import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt

def get_mean_std_results(df):
	odom_multipliers = np.unique(df['Odometry Multiplier'])
	results = []

	for multiplier in odom_multipliers:
		data = df[df['Odometry Multiplier'] == multiplier]
		Error_corrected = data['Path Error'][df['Correction']]
		Error_uncorrected = data['Path Error'][df['Correction'] == False]
		Image_corrected = data['Image Error'][df['Correction']]
		Image_uncorrected = data['Image Error'][df['Correction'] == False]

		results.append([
			np.mean(Error_corrected),
			np.std(Error_corrected, ddof=1),
			np.mean(Error_uncorrected),
			np.std(Error_uncorrected, ddof=1),
			np.mean(Image_corrected),
			np.std(Image_corrected, ddof=1),
			np.mean(Image_uncorrected),
			np.std(Image_uncorrected, ddof=1),
		])

	results = pd.DataFrame(results, columns=['Corrected Error mean','Corrected Error std','Uncorrected Error mean','Uncorrected Error std','Corrected Image Error mean','Corrected Image Error std','Uncorrected Image Error mean','Uncorrected Image Error std'], index=odom_multipliers)
	results.index.rename('Odometry Multiplier', inplace=True)

	return results

def get_theoretical_results(df, N, correction, sr, d):
	theoretical = []
	for multiplier in df.index:
		path = d / multiplier
		for i in range(1, N):
			loc = round(path/d)
			if loc < i:
				path += d * (correction ** min(sr, i-loc)) / multiplier
			else:
				path += d / multiplier
		theoretical.append(N*d - path)
	return pd.DataFrame({'Theoretical Error': theoretical}, index=df.index)

labels = ['Odometry Multiplier', 'Correction', 'Path Error', 'Image Error']
data1 = [
	[1.0, 	True, 	70,		0],
	[1.0, 	True, 	65, 	0],
	[1.0, 	True, 	50, 	0],
	[1.0, 	False, 	140, 	0],
	[1.25, 	True, 	100, 	0],
	[1.25, 	True, 	180, 	0],
	[1.25, 	True, 	110, 	0],
	[1.25, 	False, 	1080,	6*200],
	[1.5, 	True,	280, 	0],
	[1.5, 	True, 	300, 	200],
	[1.5, 	True, 	200, 	0],
	[1.5, 	False, 	1770, 	6*200],
	[1.75, 	True, 	880, 	4*200],
	[1.75, 	True, 	1180, 	4*200],
	[1.75, 	True, 	920, 	4*200],
	[1.75, 	False, 	2360, 	np.NaN]
]
data2 = [
	[1.0, 	True, 	60,		0],
	[1.0, 	True, 	75, 	0],
	[1.0, 	True, 	50, 	0],
	[1.0, 	False, 	220, 	0],
	[1.5, 	True, 	350, 	1*200],
	[1.5, 	True, 	380, 	1*200],
	[1.5, 	True, 	250, 	1*200],
	[1.5, 	False, 	1750,	8*200],
	[2.0, 	True,	495, 	2*200],
	[2.0, 	True, 	480, 	2*200],
	[2.0, 	True, 	560, 	2*200],
	[2.0, 	False, 	2520, 	11*200],
	[2.5, 	True, 	1010, 	5*200],
	[2.5, 	True, 	1400, 	7*200],
	[2.5, 	True, 	1810, 	8*200],
	[2.5, 	False, 	2990, 	14*200]
]

df1 = pd.DataFrame(data1, columns=labels)
results1 = get_mean_std_results(df1)
results1 = results1.join(get_theoretical_results(results1, N=24, correction=1.5, sr=1, d=200))

df2 = pd.DataFrame(data2, columns=labels)
results2 = get_mean_std_results(df2)
results2 = results2.join(get_theoretical_results(results2, N=24, correction=1.5, sr=2, d=200))

matplotlib.style.use('ggplot')
results1[['Corrected Error mean','Theoretical Error','Corrected Image Error mean']].plot(kind='bar', yerr=[results1['Corrected Error std'],np.zeros(4),results1['Corrected Image Error std']])
plt.xticks(rotation=0)
plt.ylabel('Path Error (mm)')
plt.legend(['Real path error','Theoretical path error','Image localisation error (at end)'], loc='upper left')
plt.title('Effect of odom corruption on localisation (SR=1)')

# plt.figure()
# matplotlib.style.use('ggplot')
results2[['Corrected Error mean','Theoretical Error','Corrected Image Error mean']].plot(kind='bar', yerr=[results2['Corrected Error std'],np.zeros(4),results2['Corrected Image Error std']])
plt.xticks(rotation=0)
plt.ylabel('Path Error (mm)')
plt.legend(['Real path error','Theoretical path error','Image localisation error (at end)'], loc='upper left')
plt.title('Effect of odom corruption on localisation (SR=2)')

plt.show()