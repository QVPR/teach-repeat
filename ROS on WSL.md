# Running ROS on Windows using Windows Subsystem for Linux

This guide has been adapted from [here](https://jack-kawell.com/2019/06/24/setting-up-a-ros-development-environment-in-windows/).

## 1. Enable WSL on Windows

Open Powershell as Administrator (right click the start menu icon -> Windows PowerShell (Admin)) and enter the following command:

```powershell
Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux
```

## 2. Download Ubuntu 18.04

Go to the Windows store, search for Ubuntu and install Ubuntu 18.04 LTS. Launch it and it should say `Installing, this may take a few minutes...` Wait for this to complete then choose a username and password.

## 3. Add WSL to Windows Security Exception list

Sometimes Windows Security can have issues scanning WSL too aggressively and slowing down performance. To get around this, you can add a security exception for WSL.

1. Open Windows Security
2. Virus & threat protection
3. Virus & threat protection settings -> Manage settings
4. Exclusions -> Add or remove exclusions
5. Add an exclusion -> Folder
6. Type in `\\wsl$\Ubuntu-18.04` in the top address bar and press enter. You should see folders such as `bin` and `boot`...
7. Select Folder

## 4. Install an X Server to enable graphical programs

Install [VcXsrv](https://sourceforge.net/projects/vcxsrv/). When starting it, make sure `Native opengl` is unchecked and `Disable access control` is checked. Before you click finish you can save the configuration and then just double click this file in future. When VcXsrv is running you should see an icon in the Windows taskbar tray. Now run the following in the terminal to enable the display.

```bash
echo "export DISPLAY=:0" >> ~/.bashrc
source ~/.bashrc
```

## 5. Installing ROS

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

## 6. Install Gazebo (the simulator)

Now that ROS is installed, Gazebo is a bit easier to install. Just run the following command and wait a few minutes.

```bash
sudo apt install ros-melodic-gazebo-dev
```

After installation, you can run gazebo by typing the following command. **Note:** the first time you start gazebo it will take a while to start because it;s downloading some basic model files.

TODO
This is currently broken - install the new version 9.14 instead from the osrf repo