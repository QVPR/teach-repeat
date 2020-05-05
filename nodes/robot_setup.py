#!/usr/bin/python

import rospy
import random
import math
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from std_msgs.msg import UInt32, Bool

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
		self.should_disable_cliff_sensors = rospy.get_param('~disable_cliff_sensors', False)

		self.ready = None

	def setup_publishers(self):
		rospy.wait_for_service('miro/control/kinematic_joints/set_fixed_state')
		rospy.wait_for_service('miro/control/kinematic_joints/fixed/enable')
		rospy.wait_for_service('miro/control/kinematic_joints/fixed/disable')
		rospy.wait_for_service('/miro/sensors/odom/reset')
		self.set_fixed_state = rospy.ServiceProxy('miro/control/kinematic_joints/set_fixed_state', SetJointState, persistent=False)
		self.enable_fixed_state = rospy.ServiceProxy('miro/control/kinematic_joints/fixed/enable', Trigger, persistent=False)
		self.disable_fixed_state = rospy.ServiceProxy('miro/control/kinematic_joints/fixed/disable', Trigger, persistent=False)
		self.reset_odom = rospy.ServiceProxy('/miro/sensors/odom/reset', Trigger, persistent=False)

		self.pub_flags = rospy.Publisher('miro/control/flags', UInt32, queue_size=0)
		self.pub_ready = rospy.Publisher("/miro/ready", Bool, queue_size=1)

	def setup_subscribers(self):
		self.sub_at_set_point = rospy.Subscriber("/miro/control/kinematic_joints/at_set_point", Bool, self.process_joint_at_set_point, queue_size=1)

	def process_joint_at_set_point(self, msg):
		if msg.data and self.ready == False:
			self.ready = True
			self.pub_ready.publish(Bool(data=True))

	def start(self):
		self.set_fixed_state(self.joint_states)
		self.enable_fixed_state()
		if self.should_reset_odom:
			self.reset_odom()
		if self.should_disable_cliff_sensors:
			# flag to disable cliff reflex, from miro constants
			self.pub_flags.publish(UInt32(data = 1 << 21))
		self.ready = False

	def stop(self):
		self.disable_fixed_state()
		self.pub_flags.publish(UInt32(data = 0)) # reenable cliff reflex

if __name__ == "__main__":
	rospy.init_node("miro_robot_setup")
	setup = miro_robot_setup()

	setup.start()
	rospy.on_shutdown(setup.stop)

	rospy.spin()