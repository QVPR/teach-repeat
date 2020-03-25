#!/usr/bin/python

import rospy
import random
import math
from sensor_msgs.msg import JointState

class miro_joint_controller:

	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):
		self.lift = math.radians(rospy.get_param('~lift', 30.0))
		self.yaw = math.radians(rospy.get_param('~yaw', 0.0))
		self.pitch = math.radians(rospy.get_param('~pitch', 8.0))

		self.joint_states = JointState()
		self.joint_states.name = ['tilt','lift','yaw','pitch']
		self.joint_states.position = [0.0, self.lift, self.yaw, self.pitch]

	def setup_publishers(self):
		self.pub_joints = rospy.Publisher("/miro/control/kinematic_joints", JointState, queue_size=0)

	def setup_subscribers(self):
		pass

	def publish_joint_state(self):
		joint_state_command = JointState()
		joint_state_command.header.stamp = rospy.Time.now()
		joint_state_command.name = self.joint_states.name
		joint_state_command.position = [joint_pos + 0.001*random.random() for joint_pos in self.joint_states.position]
		self.pub_joints.publish(joint_state_command)


if __name__ == "__main__":

	rospy.init_node("miro_joint_controller")
	controller = miro_joint_controller()
	
	rate = rospy.Rate(50)
	while not rospy.is_shutdown():
		controller.publish_joint_state()
		rate.sleep()

