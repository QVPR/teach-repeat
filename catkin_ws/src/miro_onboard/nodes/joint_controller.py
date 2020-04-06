#!/usr/bin/python

# Note: Should be run onboard Miro

import rospy
import random
import math
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from miro_onboard.srv import SetJointState

class miro_joint_controller:

	def __init__(self):
		self.setup_parameters()
		self.setup_publishers()
		self.setup_subscribers()

	def setup_parameters(self):
		self.joint_states = None
		self.enabled = False

	def setup_publishers(self):
		self.pub_joints = rospy.Publisher("/miro/control/kinematic_joints", JointState, queue_size=0)

	def setup_subscribers(self):
		self.srv_set_joint_states = rospy.Service('miro/control/kinematic_joints/set_fixed_state', SetJointState, self.set_joint_state)
		self.srv_enable_set_joint_states = rospy.Service('miro/control/kinematic_joints/fixed/enable', Trigger, self.enable_set_joint_state)
		self.srv_disable_set_joint_states = rospy.Service('miro/control/kinematic_joints/fixed/disable', Trigger, self.disable_set_joint_state)

	def set_joint_state(self, srv):
		self.joint_states = srv.jointStates
		return (True, "")
	
	def enable_set_joint_state(self, srv):
		if self.joint_states is not None:
			self.enabled = True
			return (True, "")
		else:
			error_message = "No desired joint state set: call miro/control/kinematic_joints/set_fixed_state service first."
			return (False, error_message)

	def disable_set_joint_state(self, srv):
		self.enabled = False
		return (True, "")

	def publish_joint_state(self):
		if self.enabled:
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

