# Ubuntu on Windows using a Virtual Machine

This is a moderately difficult installation process but will allow you to run a full Ubuntu installation in Windows. You will need 10-20 GB of hard drive space and a lot of RAM (>=8 GB) for reasonable performance (since you're running two OSes at once). **Note:** running the Gazebo simulation in a virtual machine has been known to cause difficulties. It might work, but I wouldn't really recommend it.

## 1. Install Virtualbox

To run a virtual machine you need a virtual machine manager. You can install [Virtualbox](https://www.virtualbox.org/wiki/Downloads) for free (or any other VM client you prefer, but the installation process will vary).

## 2. Installing Ubuntu

Download the .iso image for Ubuntu 18.04 [here](http://releases.ubuntu.com/18.04/) - select `64-bit PC (AMD64) desktop image`.

Now follow the installation instructions [here](https://linuxhint.com/install_ubuntu_18-04_virtualbox/). Then install guest additions by going to Devices -> Install Guest Additions. If that fails more details are provided [here](https://askubuntu.com/questions/22743/how-do-i-install-guest-additions-in-a-virtualbox-vm).
