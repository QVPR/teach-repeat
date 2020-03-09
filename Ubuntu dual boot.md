# Native Ubuntu under dual boot on Windows

This is the best approach to natively run Ubuntu under dual boot, but has the most difficult installation process. If you need to do a lot of work with the simulator this will give you by far the best performance. You will need at least 20 GB of hard drive space to install.

## Preparing for Installation

Download the .iso image for Ubuntu 18.04 [here](http://releases.ubuntu.com/18.04/) - select `64-bit PC (AMD64) desktop image`. You will need to burn this image onto a USB drive using [Rufus](https://rufus.ie/). Select the following options:

* Device - select your USB (**Note:** this will completely wipe everything off the USB so make sure you take a backup)
* Boot selection - Disk or ISO image, then click SELECT and find your Ubuntu .iso file
* Default options for the rest should be fine - click START

## Installing Ubuntu

Follow the step-by-step guide [here](https://www.tecmint.com/install-ubuntu-alongside-with-windows-dual-boot/). Be sure to select `Install Ubuntu alongside Windows 10`, otherwise your windows installation will be erased!
