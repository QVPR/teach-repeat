#!/bin/bash

read -p "Miro IP: " ip
echo "Copying files..."
scp -r "$(readlink -f $(dirname "$0")/..)" "miro@$ip:/home/miro/mdk/catkin_ws/src"
scp -r "$(readlink -f $(dirname "$0")/../../miro_onboard_packages)" "miro@$ip:/home/miro/mdk/catkin_ws/src"
echo "Building files..."
ssh -t "miro@$ip" bash -c "'
cd ~/mdk/catkin_ws
source /opt/ros/kinetic/setup.bash
catkin_make -j1 -DCATKIN_BLACKLIST_PACKAGES=\"ps3joy;spacenav_node;wiimote\"
'"
# -j1 slows the compilation, but stops the SSH link freezing when all processors are in use for make
echo "Configuring startup behaviour..."
ssh -t "miro@$ip" bash -c "'
cd ~/.miro2/config
echo \"source ~/mdk/catkin_ws/devel/setup.bash && roslaunch --wait miro_onboard onboard.launch &\" > on_system_ready.sh
chmod +x on_system_ready.sh
'"