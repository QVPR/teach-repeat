#!/bin/bash

read -p "Miro IP: " ip
echo "Copying files..."
rsync -avuzh "$(readlink -f $(dirname "$0")/..)" "miro@$ip:/home/miro/mdk/catkin_ws/src"
rsync -avuzh "$(readlink -f $(dirname "$0")/../../miro_onboard_packages)" "miro@$ip:/home/miro/mdk/catkin_ws/src"
rsync -avuzh "$(readlink -f $(dirname "$0")/../../miro_teach_repeat)" "miro@$ip:/home/miro/mdk/catkin_ws/src"
echo "Configuring startup behaviour..."
ssh -t "miro@$ip" bash -c "'
cd ~/.miro2/config
echo \"source ~/mdk/catkin_ws/devel/setup.bash && roslaunch --wait miro_onboard onboard.launch &\" > on_system_ready.sh
chmod +x on_system_ready.sh
'"
echo "Building files..."
echo "Build may fail due to memory usage. Try killing memory intensive processes, building, then restarting Miro."
ssh -t "miro@$ip" bash -c "'
cd ~/mdk/catkin_ws
source /opt/ros/kinetic/setup.bash
touch src/miro_onboard_packages/joystick_drivers/ps3joy/CATKIN_IGNORE
touch src/miro_onboard_packages/joystick_drivers/spacenav_node/CATKIN_IGNORE
touch src/miro_onboard_packages/joystick_drivers/wiimote/CATKIN_IGNORE
catkin build -j1
'"
# -j1 slows the compilation, but stops the SSH link freezing when all processors are in use for make
# also keeps memory usage low so there is less change of it hanging forever