#!/bin/bash

CONTROLLER_MAC=5C:BA:37:CE:CC:E2
CONTROLLER_DEV=/dev/input/js0

read -p "Miro IP: " ip
echo "Connecting bluetooth controller..."
ssh -t "miro@$ip" bash -c "'
sudo bash -c \"echo 1 > /sys/module/bluetooth/parameters/disable_ertm\"

while [ ! -e $CONTROLLER_DEV ]
do
	echo \"====================\"
	echo \"Controller not connected - trying to connect\"
	<<< \"connect $CONTROLLER_MAC
	quit\" sudo bluetoothctl

	sleep 5s
done
echo \"Controller connected\"
sudo chmod a+r $CONTROLLER_DEV
'"