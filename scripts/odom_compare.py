#!/usr/bin/python

import numpy as np
import cv2
import os
import pickle
import time
import json
import math
import sys
import matplotlib.pyplot as plt

def read_file(filename):
	with open(filename, 'r') as f:
		data = f.read()
	return data

dir1 = os.path.expanduser('~/miro/data/manual_e1/')
dir2 = os.path.expanduser('~/miro/data/manual_e2/')

pose_files1 = [dir1+f for f in os.listdir(dir1) if f[-9:] == '_pose.txt']
pose_files1.sort()
pose_files2 = [dir2+f for f in os.listdir(dir2) if f[-9:] == '_pose.txt']
pose_files2.sort()

poses1 = [json.loads(read_file(p)) for p in pose_files1]
poses2 = [json.loads(read_file(p)) for p in pose_files2]

xs1 = [p['position']['x'] for p in poses1]
ys1 = [p['position']['y'] for p in poses1]
xs2 = [p['position']['x'] for p in poses2]
ys2 = [p['position']['y'] for p in poses2]

SET_SAME_ORIGIN = True
if SET_SAME_ORIGIN:
	xo = xs2[0]
	yo = ys2[0]
	xs2 = [x - xo for x in xs2]
	ys2 = [y - yo for y in ys2]

fig,ax = plt.subplots()
ax.scatter(xs1, ys1, c='#00ff00')
ax.scatter(xs2, ys2, c='#0000ff')

ax.scatter(xs1[0], ys1[0], s=100, c='#000000', marker='x')
ax.scatter(xs2[0], ys2[0], s=100, c='#000000', marker='x')


plt.show()
