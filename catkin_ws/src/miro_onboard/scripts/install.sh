#!/bin/bash

read -p "Miro IP: " ip
echo "Copying files..."
scp -r "$(readlink -f $(dirname "$0")/..)" "miro@$ip:/home/miro/mdk/catkin_ws/src"
scp -r "$(readlink -f $(dirname "$0")/../../miro_onboard_packages)" "miro@$ip:/home/miro/mdk/catkin_ws/src"
echo "Building files..."
ssh -t "miro@$ip" "cd ~/mdk/catkin_ws && source /opt/ros/kinetic/setup.bash && catkin_make -j1"
echo "Configuring startup behaviour..."
ssh -t "miro@$ip" "cd ~/.miro2/config && echo 'source ~/mdk/catkin_ws/devel/setup.bash && roslaunch --wait miro_onboard onboard.launch &' > on_system_ready.sh && chmod +x on_system_ready.sh"