# Running ROS on Windows using Windows Subsystem for Linux

This is the easiest installation option on Windows and uses the least hard drive space (~3.5 GB). The performance is fine for standard ROS tasks but not very good for the simulator (because WSL is unable to use the system's GPU for acceleration). You might also run into random bugs from time to time, but things are now quite stable.

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

Make sure you start VcXsrv whenever you restart your computer if you want to use graphical programs in Ubuntu, otherwise you'll get an error.
