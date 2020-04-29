#!/usr/bin/python

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
		self.K_P = 0.2

	def setup_publishers(self):
		self.pub_joints = rospy.Publisher("/miro/control/kinematic_joints", JointState, queue_size=1)

	def setup_subscribers(self):
		self.sub_joints = rospy.Subscriber("/miro/sensors/kinematic_joints", JointState, self.process_joint_data, queue_size=1)
		self.srv_set_joint_states = rospy.Service('miro/control/kinematic_joints/set_fixed_state', SetJointState, self.set_joint_state)
		self.srv_enable_set_joint_states = rospy.Service('miro/control/kinematic_joints/fixed/enable', Trigger, self.enable_set_joint_state)
		self.srv_disable_set_joint_states = rospy.Service('miro/control/kinematic_joints/fixed/disable', Trigger, self.disable_set_joint_state)

	def set_joint_state(self, srv):
		self.joint_states = srv.jointStates
		print(self.joint_states.position)
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

	def process_joint_data(self, msg):
		if self.enabled:
			errors = [x-y for x, y in zip(self.joint_states.position, msg.position)]
			errors = [error if abs(error) > 0.1 else 0.0 for error in errors]
			set_joint_positions = [joint_position + self.K_P*error + random.random()*0.01 for error, joint_position in zip(errors, self.joint_states.position)]

			joint_state_command = JointState()
			joint_state_command.header.stamp = rospy.Time.now()
			joint_state_command.name = self.joint_states.name
			joint_state_command.position = set_joint_positions
			self.pub_joints.publish(joint_state_command)


if __name__ == "__main__":
	rospy.init_node("miro_joint_controller")
	controller = miro_joint_controller()
	rospy.spin()

