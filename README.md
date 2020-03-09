# Miro

Info about how to setup and use the Miro robot, and some code examples.

![miro](http://labs.consequentialrobotics.com/miro-e/docs/media/icon_intro_h300.jpg)

## Overview

Documentation about Miro can be found at [this](http://labs.consequentialrobotics.com/miro-e/docs/index.php?page=Introduction) website. For the most part it's quite detailed and covers everything.

### Specs

Miro's specs can be found [here](http://labs.consequentialrobotics.com/miro-e/docs/index.php?page=Introduction_Specification). The main specs are:

| Category | Details
| -------- | -------
| Robot    | 3.3 kg; differential drive; top speed = 0.4 m/s
| Power    | 4.8 V / 10 Ah; battery life: ~6 hours (active) - 12 hours (idle)
| Cameras  | 2 cams; up to 1280x720 @ 15 fps; FOV each = 120/62; FOV total = ~180/62 (60 degree overlap)
| Sonar    | 3 cm - 1 m range (on nose)
| Other sensors | Mic, Touch (head and back), light, cliff, wheel encoders, IMU, joint positions
| Compute  | Raspberry Pi 3B+ (Quad Core ARM A53 @ 1.4 GHz, 1 GB RAM, 16 GB storage, Bluetooth, WiFi, USB)
| Actuators| Neck (3 DOF), Tail (2 DOF), Ears (rotate), Eyelids (open/close), LEDs (6 x RGB), Speaker

## Development Environment

Miro comes with ROS installed (kinetic) which is the best for development. ROS can be run on- or offboard the robot, as can code controlling the robot. There is also a simulation environment using Gazebo.

Here are some basic steps to setup the development environment:

1. Install [Ubuntu](https://ubuntu.com/download/desktop) 16.04 or 18.04. (Installing in a VM can apparently cause issue with the simulator, so best not to)
2. Install ROS [Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) with 16.04 or [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) with 18.04 (best to choose the desktop install)
3. Install Gazebo (if you would like the simulator) `sudo apt install ros-kinetic-gazebo-dev` or `sudo apt install ros-melodic-gazebo-dev`
4. Install the [MDK](http://labs.consequentialrobotics.com/miro-e/docs/index.php?page=Developer_Install_Steps_Install_MDK) (Miro Development Kit)
5. (Optional but helpful) Install the [Miro app](http://labs.consequentialrobotics.com/miro-e/docs/index.php?page=Husbandry_MIROapp). This lets you connect to Miro through Bluetooth to initially configure its network settings, and do other things like run demos. Note that the app is only available for relatively recent Android devices.

## Using the Robot

### Startup

There's a very detailed explanation of the startup and shutdown procedures for Miro [here](http://labs.consequentialrobotics.com/miro-e/docs/index.php?page=Husbandry_Power_Up). The main points being:

* Make sure the robot is OFF before charging it (it will make a "satellite noise" when the batteries are low)
* The power switch is on the base (next to the battery pack)
* If configured correctly, Miro will automatically connect to the QUT network and read out its ip address on startup. (controlled by the `MIRO_SPEAK_IP` setting in `/home/miro/.miro2/config/user_setup.bash` or through the app)
  * **Note:** this doesn't work if Miro is configured to connect to an external ROS master

### Connecting and running code

You can connect to Miro through [SSH](http://labs.consequentialrobotics.com/miro-e/docs/index.php?page=Technical_Interfaces_SSH). The username and password are both `miro`.

```bash
ssh miro@172.19.x.x
miro@172.19.x.x's password: miro
```

Some basic demos are provided you can run on the robot (`mdk/bin/shared` folder).

```bash
### Test the actuators
~/mdk/bin/shared/client_test.py workout
# or
~/mdk/bin/shared/client_test.py spin
# you can see the full list of demos by running client_test.py without any arguments
~/mdk/bin/shared/client_test.py

### Test movement (press Ctrl+C to exit)
~/mdk/bin/shared/client_manual.py

### Test speech (like Miro reading its IP - can only be numbers and . and should be preceded by an underscore)
~/mdk/bin/shared/client_stream.py _1234.321
```

### Running Code Remotely

With Miro powered on and connected to the network, you can also run code on your local machine to control Miro. For example, the GUI lets you view most of Miro's sensors and control the various actuators.

```bash
# change this location if you didn't install the MDK to your home directory
cd ~/mdk/bin/shared
./client_gui.py
```
