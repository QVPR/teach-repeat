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

## 5. Installing ROS (Robot Operating System)

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

As with ROS, run the following commands in the terminal to install Gazebo.

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-gazebo-dev
# Wait a bit (~5 min)
```

After installation, you can run gazebo by typing the following command.

```bash
gazebo
```

## 7. Install the MDK

First, install the dependencies for the MDK.

```bash
sudo apt install python-matplotlib python-tk python-gi python-gi-cairo
```

Now, download the MDK by running the following command in the terminal (you can change the name parameter after `--data` if you want).

```bash
curl 'http://labs.consequentialrobotics.com/download.php?file=mdk_2-200131.tgz' -H 'User-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64; rv:74.0) Gecko/20100101 Firefox/74.0' -H 'Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/webp,*/*;q=0.8' -H 'Accept-Language: en-US,en;q=0.5' --compressed -H 'Referer: http://labs.consequentialrobotics.com/download.php?file=mdk_2-200131.tgz' -H 'Content-Type: application/x-www-form-urlencoded' -H 'Origin: http://labs.consequentialrobotics.com' -H 'DNT: 1' -H 'Connection: keep-alive' -H 'Upgrade-Insecure-Requests: 1' --data 'name=test&org=QUT&email=&agree=on' --output mdk.tgz
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

Now install the MDK with the following commands.

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

You should see a simulation of Miro and a blue ball open in Gazebo. Unfortunately, the performance is likely to be very slow because WSL isn't able to access the GPU for improved rendering performance.
