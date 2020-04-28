#!/usr/bin/python

import os
import subprocess

from enum import Enum

import rospy
from std_msgs.msg import UInt16

# this enum just provides a convenient way of storing the indices for each touch sensor
class HeadTouchSensor(Enum):
	nose_mid_left = 0
	nose_far_left = 1
	forehead_left = 2
	ear_left_front = 3
	ear_left_left = 4
	ear_left_right = 5
	ear_left_back = 6
	nose_mid_right = 7
	nose_far_right = 8
	forehead_right = 9
	ear_right_front = 10
	ear_right_right = 11
	ear_right_left = 12
	ear_right_back = 13

class miro_ontouch_ip:

	def __init__(self):
		# subscribe to the touch sensor message
		self.sub_touch_body = rospy.Subscriber("/miro/sensors/touch_head", UInt16, self.got_touch_sensor_data, queue_size=1)

	def got_touch_sensor_data(self, msg):
		# partly documented here: http://labs.consequentialrobotics.com/miro-e/docs/index.php?page=Technical_Interfaces_ROS
		# and from experimentation with the GUI - data is a 16 bit int where the lower 14 bits give the state of each head sensor
		# put the touch sensor values in a nice format so we can query them:
		touch_sensor = {}
		for element in HeadTouchSensor:
			touch_sensor[element.name] = bool(msg.data & (1 << element.value))

		# if all four front head sensors are pressed, read the IP
		if touch_sensor["nose_mid_left"] and touch_sensor["nose_mid_right"] and touch_sensor["nose_far_left"] and touch_sensor["nose_far_right"]:
			# hacky way to get this to run...
			subprocess.call(['./client_stream.py','_' + os.environ['ROS_IP']], cwd='/home/miro/mdk/bin/shared')

if __name__ == "__main__":
	# intialise this program as a ROS node
	rospy.init_node("miro_ontouch_ip")
	# start the demo program
	demo = miro_ontouch_ip()
	# spin makes the program run while ROS is running
	rospy.spin()