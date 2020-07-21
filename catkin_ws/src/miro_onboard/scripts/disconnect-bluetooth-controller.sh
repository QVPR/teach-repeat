#!/bin/bash

CONTROLLER_MAC=5C:BA:37:CE:CC:E2
CONTROLLER_DEV=/dev/input/js0

read -p "Miro IP: " ip
echo "Disonnecting bluetooth controller..."
ssh -t "miro@$ip" bash -c "'
while [ -e $CONTROLLER_DEV ]
do
	echo \"====================\"
	echo \"Controller connected - trying to disconnect\"
	<<< \"disconnect $CONTROLLER_MAC
	quit\" sudo bluetoothctl

	sleep 5s
done
echo \"Controller disconnected\"
'"