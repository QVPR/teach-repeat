#!/usr/bin/python

import rospy
import random
import math
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from miro_onboard.srv import SetJointState

class miro_robot_setup:

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

		self.should_reset_odom = rospy.get_param('~reset_odom', False)

	def setup_publishers(self):
		rospy.wait_for_service('miro/control/kinematic_joints/set_fixed_state')
		rospy.wait_for_service('miro/control/kinematic_joints/fixed/enable')
		rospy.wait_for_service('miro/control/kinematic_joints/fixed/disable')
		rospy.wait_for_service('/miro/sensors/odom/reset')
		self.set_fixed_state = rospy.ServiceProxy('miro/control/kinematic_joints/set_fixed_state', SetJointState, persistent=False)
		self.enable_fixed_state = rospy.ServiceProxy('miro/control/kinematic_joints/fixed/enable', Trigger, persistent=False)
		self.disable_fixed_state = rospy.ServiceProxy('miro/control/kinematic_joints/fixed/disable', Trigger, persistent=False)
		self.reset_odom = rospy.ServiceProxy('/miro/sensors/odom/reset', Trigger, persistent=False)

	def setup_subscribers(self):
		pass

	def start(self):
		self.set_fixed_state(self.joint_states)
		self.enable_fixed_state()
		if self.should_reset_odom:
			self.reset_odom()

	def stop(self):
		self.disable_fixed_state()

if __name__ == "__main__":
	rospy.init_node("miro_robot_setup")
	setup = miro_robot_setup()

	setup.start()
	rospy.on_shutdown(setup.stop)

	rospy.spin()

