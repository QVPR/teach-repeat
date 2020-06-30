import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import collections

# 29th June 12624aba95a52a0ada489526439c33fc21fa6126
# K = 0.01, K2 = 0.05
# Note: corrections weren't properly scaled with the image size, 11 px image was 1/10th gain of normal
# data_arvo = collections.OrderedDict((
# 	('(28x11)', {'pixel', [1, 1, 1, 1, 1], 'subpixel', [1, 1, 1, 1, 1]}),
# 	('(14x5)', {'pixel', [0, 1, 0, 0, 1], 'subpixel', [1, 1, 1, 1, 1]}),
# 	('(7x2)', {'pixel', [1, 1, 1, 1, 1], 'subpixel', [1, 1, 1, 1, 1]})
# ))

# data_arvo = { '(28x11)': {'pixel': [1, 1, 1, 1, 1], 'subpixel': [1, 1, 1, 1, 1]},
# 	 '(14x5)': {'pixel': [0, 1, 0, 0, 1], 'subpixel': [1, 1, 1, 1, 1]},
# 	 '(7x2)': {'pixel': [1, 1, 1, 1, 1], 'subpixel': [1, 1, 1, 1, 1]}
# }

data_arvo = collections.OrderedDict(( 
	('pixel' , collections.OrderedDict((
		('(28x11)', 1),
		('(14x5)', 0.4),
		('(7x2)', 1)
	))),
	('subpixel' , collections.OrderedDict((
		('(28x11)', 1),
		('(14x5)', 1),
		('(7x2)', 1)
	)))
))

data_morning = collections.OrderedDict(( 
	('pixel' , collections.OrderedDict((
		('(28x11)', 1),
		('(14x5)', 0.8),
		('(7x2)', 0)
	))),
	('subpixel' , collections.OrderedDict((
		('(28x11)', 0.4),
		('(14x5)', 0.6),
		('(7x2)', 0)
	)))
))

df_arvo = pd.DataFrame(data_arvo)
df_morning = pd.DataFrame(data_morning)

matplotlib.style.use('ggplot')
fig, (ax1, ax2) = plt.subplots(2,1, sharex=True)
(100*df_arvo.iloc[[1,0,2]]).plot(kind='bar', ax=ax1)
(100*df_morning.iloc[[1,0,2]]).plot(kind='bar', ax=ax2)



plt.xticks(rotation=0)
ax1.set_ylabel('% successful tests')
ax2.set_ylabel('% successful tests')
# plt.ylim((0,1))
ax2.set_xlabel('Image size')
ax1.set_title('Afternoon test [N=5]')
ax2.set_title('Morning test [N=5]')

plt.tight_layout()

plt.show()