# Troubleshooting

## Miro isn't turning on

This occurs because Miro's charging port disconnects the battery power from the robot when a charger is plugged in. It's possible for the contact in the charging port to get stuck, keeping the battery disconnected and preventing the robot from turning on.

The charger cable just fits under the robot normally, but after significant use the plastic caster at the back can be worn down, reducing the height of the robot. This causes Miro to rest on the charging cable itself, which seems to be responsible for the battery port issue above.

### Solution

* Stick a sharp object (screw driver / skewer) into the charging point and press the contact down, try to free it up and Miro will then turn on
* Make sure Miro isn't resting on the charging cable while charging
* Replace the caster (3D print a new one - model available [here](hardware/Miro-caster.STL))

## Miro won't boot up, two blue lights slowly flashing

This means Miro's onboard computer (Raspberry Pi) isn't booting up successfully. The cause of this isn't completely clear, but it has been triggered by crashing the SSH connection to Miro when running `catkin_make` using all cores (the default).

### Solution

No proper fix found. But turning Miro off, charging the battery, then restarting fixed the issue.

## Miro's wheel keeps locking up when driving

This is because of the cliff sensors causing Miro to stop, often triggered by the black carpet in QUT S11. Testing should show that the commanded wheel speed in the `/miro/sensors/wheel_speed_cmd` topic is getting set to zero. If this is not the case, a different issue is occurring.

### Solution

A flag can be published to Miro to disable the cliff sensors as detailed [here](http://labs.consequentialrobotics.com/miro-e/docs/index.php?page=Technical_Interfaces_ROS#control/flags). I couldn't get the persistent mode to work, so the example in [miro_setup.py](catkin_ws/src/miro_teach_repeat/nodes/miro_setup.py) continually publishes the flag.

## Holding Miro's joints at a set position doesn't work

When Miro reaches the commanded joint position, power is cut to the motors. For the head joints, this doesn't work so well because movement of the base causes the head to slowly tilt forwards. The head is counterweighted by a spring in the body, but it seems this isn't stiff enough to completely support it.

Continually sending the same joint command to Miro doesn't work either. It seems that Miro's joint control checks whether commands are repeated, and if so, ignores them. It doesn't seem this is based on the actual position of the joints, because no movement is made even if the joints are physically moved between repeated sending of the same command.

### Solution

It's not very nice, but send a continual stream of joint commands with enough random noise added on to overcome the ignoring of repeated messages. And example of this is shown in [joint_controller.py](catkin_ws/src/miro_onboard/nodes/joint_controller.py).
