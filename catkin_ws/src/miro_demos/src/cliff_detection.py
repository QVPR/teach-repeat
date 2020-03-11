#!/usr/bin/python

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped

# Note: the program is only meant to be run with the Miro gazebo simulator!
# check Simulator.md to see more about tuning these parameters

CLIFF_THRESHOLD = 0.1
REVERSE_TIME = 0.1
SPIN_TIME = 0.5

REVERSE_SPEED = -0.2 #m/s
FORWARDS_SPEED = 0.4 #m/s
SPIN_SPEED = 2 #rad/s

class miro_cliff_detection:

	def __init__(self):
		self.detected_cliff = False
		# subscribe to the touch sensor message
		self.sub_cliff_sensor = rospy.Subscriber("/miro/sensors/cliff", Float32MultiArray, self.got_cliff_sensor_data, queue_size=1)
		# publish motor commands
		self.pub_cmd_vel = rospy.Publisher("/miro/control/cmd_vel", TwistStamped, queue_size=0)

	def got_cliff_sensor_data(self, msg):
		# Touch sensor values: 1 = ground, 0 = no ground
		CLIFF_THRESHOLD = 0.5

		# if we just detect a cliff, don't worry about detecting it again
		if self.detected_cliff:
			return

		if msg.data[0] < CLIFF_THRESHOLD or msg.data[1] < CLIFF_THRESHOLD:
			self.detected_cliff = True
			self.cliff_detection_time = rospy.Time.now()

	def publish_motor_speeds(self):
		velocity_message = TwistStamped()
		velocity_message.header.stamp = rospy.Time.now()

		if self.detected_cliff:
			time_since_detection = (rospy.Time.now() - self.cliff_detection_time).to_sec()
			if time_since_detection < REVERSE_TIME:
				# reverse
				velocity_message.twist.linear.x = REVERSE_SPEED
			elif time_since_detection < (REVERSE_TIME + SPIN_TIME):
				# spin
				velocity_message.twist.angular.z = SPIN_SPEED
			else:
				# reset
				self.detected_cliff = False
				velocity_message.twist.linear.x = FORWARDS_SPEED
		else:
			# drive straight
			velocity_message.twist.linear.x = FORWARDS_SPEED

		self.pub_cmd_vel.publish(velocity_message)


if __name__ == "__main__":
	# intialise this program as a ROS node
	rospy.init_node("miro_cliff_detection")
	# start the demo program
	demo = miro_cliff_detection()
	# publish the desired motor speeds at 50 Hz
	rate = rospy.Rate(50)
	while not rospy.is_shutdown():
		demo.publish_motor_speeds()
		rate.sleep()