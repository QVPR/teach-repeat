# Troubleshooting

## Miro isn't turning on

This occurs because Miro's charging port disconnects the battery power from the robot when a charger is plugged in. It's possible for the contact in the charging port to get stuck, keeping the battery disconnected and preventing the robot from turning on.

The charger cable just fits under the robot normally, but after significant use the plastic caster at the back can be worn down, reducing the height of the robot. This causes Miro to rest on the charging cable itself, which seems to be responsible for the battery port issue above.

### Solution

* Stick a sharp object (screw driver / skewer) into the charging point and press the contact down, try to free it up and Miro will then turn on
* Make sure Miro isn't resting on the charging cable while charging
* Replace the caster (3D print a new one)

## Miro won't boot up, two blue lights slowly flashing

This means Miro's onboard computer (Raspberry Pi) isn't booting up successfully. The cause of this isn't completely clear, but it has been triggered by crashing the SSH connection to Miro when running `catkin_make` using all cores (the default).

### Solution

No proper fix found. But turning Miro off, charging the battery, then restarting fixed the issue.
