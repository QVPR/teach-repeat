# Install Software for Miro Development

## Overview

Miro runs with the following software:

* [ROS](http://wiki.ros.org) (Robot Operating System) - this manages all of Miro's sensors and actuators and allows easy structuring of software. Very commonly used in robotics.
* [Gazebo](http://gazebosim.org/) - software for simulating robots that also can connect to ROS. Allows running the same code in simulation or on the robot.
* [MDK](http://labs.consequentialrobotics.com/miro-e/docs/index.php?page=Developer_Install_Steps_Install_MDK) (Miro Development Kit) - custom software provided by the creators of Miro to connect to and communicate with all Miro's sensors and actuators.

## 1. Installing Ubuntu

This software requires a linux operating system to run (most commonly Ubuntu). If you don't already have access to one, there are a number of options:

* [Install Ubuntu natively](Ubuntu%20dual%20boot.md) on your computer using dual boot (most complicated installation but best development environment - can't run at the same time as Windows)
* [Install Ubuntu in a virtual machine](Ubuntu%20on%20VM.md) (moderately easy to setup but can cause issues with the simulator)
* [Install Ubuntu via Windows Subsystem for Linux](Ubuntu%20on%20WSL.md) (easiest setup but the simulator runs a bit slowly - least well supported)

The best option depends on your computer / what work you'd like to do with Miro. Once you've installed Ubuntu, continue with the rest of the installation process.

## 2. Installing ROS (Robot Operating System)

In Ubuntu, run the following commands in order. **Note:** In the Ubuntu terminal, right click = paste.

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# you'll probably have to type in your password now
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
# wait a bit (~30 s)
sudo apt install ros-melodic-desktop
# wait a while (~1 hour)...
# If you receive a prompt about automatically restarting services, select Yes
```

Now ROS is installed! Run a few extra setup commands to make life easier.

```bash
sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

You can test whether everything is OK by running the following command. You should see the graphical window for `rviz` appear.

```bash
rviz
```

## 3. Installing Gazebo (the simulator)

As with ROS, run the following commands in the terminal to install Gazebo.

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-gazebo-dev
# Wait a bit (~5 min)
```

After installation, you can run gazebo by typing the following command. You should see the gazebo simulator startup in a blank world.

```bash
gazebo
```

## 4. Installing the MDK

First, install the dependencies for the MDK.

```bash
sudo apt install python-matplotlib python-tk python-gi python-gi-cairo
```

Now, download the MDK by running the following command in the terminal (you can change the name parameter after `--data` if you want). You can alternatively download the file in the browser [here](http://labs.consequentialrobotics.com/download.php?file=mdk_2-200131.tgz).

```bash
curl 'http://labs.consequentialrobotics.com/download.php?file=mdk_2-200131.tgz' -H 'User-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64; rv:74.0) Gecko/20100101 Firefox/74.0' -H 'Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/webp,*/*;q=0.8' -H 'Accept-Language: en-US,en;q=0.5' --compressed -H 'Referer: http://labs.consequentialrobotics.com/download.php?file=mdk_2-200131.tgz' -H 'Content-Type: application/x-www-form-urlencoded' -H 'Origin: http://labs.consequentialrobotics.com' -H 'DNT: 1' -H 'Connection: keep-alive' -H 'Upgrade-Insecure-Requests: 1' --data 'name=QUT&org=QUT&email=&agree=on' --output mdk.tgz
```

Now unzip the MDK.

```bash
tar -xzf mdk.tgz
```

Modify one of the configuration files for our ROS version.

```bash
nano mdk-200131/share/config/user_setup.bash
# scroll down with the down arrow and change the following line
export MIRO_ROS_RELEASE=kinetic
# to
export MIRO_ROS_RELEASE=melodic
# Press Ctrl+o, ENTER, Ctrl+x
```

Now install the MDK with the following commands (setting up for gazebo9).

```bash
cd mdk-200131/bin/deb64
mv libmiro_gazebo.so libmiro_gazebo7.so
mv libmiro_gazebo9.so libmiro_gazebo.so
./install_mdk.sh
```

It should end with:

```bash
All operations completed successfully.
```

You can test the installation by running the following command.

```bash
~/mdk/sim/launch_sim.sh
```

You should see a simulation of Miro and a blue ball open in Gazebo. Performance will vary depending on your Ubuntu installation.
