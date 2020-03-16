#!/bin/bash 
docker run -it \
--gpus="all" \
--env="DISPLAY"  \
--env="QT_X11_NO_MITSHM=1"  \
--net=host \
--workdir="/home/$USER" \
--volume="/home/$USER:/home/$USER" \
--volume="/etc/group:/etc/group:ro" \
--volume="/etc/passwd:/etc/passwd:ro" \
--volume="/etc/shadow:/etc/shadow:ro" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
--volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
-e LOCAL_USER_ID=`id -u $USER` \
-e LOCAL_GROUP_ID=`id -g $USER` \
-e LOCAL_GROUP_NAME=`id -gn $USER` \
miro
bash

# TODO - gazebo won't display properly...