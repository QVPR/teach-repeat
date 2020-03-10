#!/usr/bin/python

from enum import Enum

import rospy
from std_msgs.msg import UInt16
from geometry_msgs.msg import TwistStamped

class BodyTouchSensor(Enum):
	mid_front = 0
	mid_front_mid = 1
	mid_back_mid = 2
	mid_back = 3
	right_front = 4
	right_front_mid = 5
	right_mid = 6
	right_back_mid = 7
	right_back = 8
	left_front = 9
	left_front_mid = 10
	left_mid = 11
	left_back_mid = 12
	left_back = 13

class miro_touch_control:

	def __init__(self):
		# subscribe to the touch sensor message
		self.sub_touch_body = rospy.Subscriber("/miro/sensors/touch_body", UInt16, self.got_touch_sensor_data, queue_size=1)
		# publish motor commands
		self.pub_cmd_vel = rospy.Publisher("/miro/control/cmd_vel", TwistStamped, queue_size=0)

	def got_touch_sensor_data(self, msg):
		# partly documented here: http://labs.consequentialrobotics.com/miro-e/docs/index.php?page=Technical_Interfaces_ROS
		# and from experimentation with the GUI - data is a 16 bit int where the lower 14 bits give the state of each body sensor
		# put the touch sensor values in a nice format so we can query them:
		touch_sensor = {}
		for element in BodyTouchSensor:
			touch_sensor[element.name] = bool(msg.data & (1 << element.value))

		# depending on the touch sensor readings, set the velocity command message for Miro
		velocity_message = TwistStamped()
		velocity_message.header.stamp = rospy.Time.now()

		if touch_sensor["mid_front"] or touch_sensor["mid_front_mid"]:
			velocity_message.twist.linear.x = 0.2
		elif touch_sensor["mid_back"] or touch_sensor["mid_back_mid"]:
			velocity_message.twist.linear.x = -0.2

		if touch_sensor["left_mid"] or touch_sensor["left_front_mid"] or touch_sensor["left_back_mid"]:
			velocity_message.twist.angular.z = 2
		elif touch_sensor["right_mid"] or touch_sensor["right_front_mid"] or touch_sensor["right_back_mid"]:
			velocity_message.twist.angular.z = -2

		self.pub_cmd_vel.publish(velocity_message)

if __name__ == "__main__":
	# intialise this program as a ROS node
	rospy.init_node("miro_touch_control")
	# start the demo program
	demo = miro_touch_control()
	# spin makes the program run while ROS is running
	rospy.spin()