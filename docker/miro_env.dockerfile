FROM ros:melodic-ros-base-bionic

RUN apt-get update && apt-get install -y \
    lsb-release \
    curl \
    iputils-ping \
    net-tools \
    wget \
    && rm -rf /var/lib/apt/lists/*

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# install gazebo
RUN apt-get update && apt-get install -y\
    gir1.2-gtk-3.0 \
    gobject-introspection \
    python-catkin-tools \
    python-matplotlib \
    python-gi \
    python-gi-cairo \
    python-tk \
    ros-melodic-gazebo-dev \
    && rm -rf /var/lib/apt/lists/*

RUN addgroup --system --gid 1000 miro && adduser --system --uid 1000 --gid 1000 miro
USER miro
WORKDIR /home/miro

RUN curl 'http://labs.consequentialrobotics.com/download.php?file=mdk_2-200131.tgz' -H 'User-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64; rv:74.0) Gecko/20100101 Firefox/74.0' -H 'Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/webp,*/*;q=0.8' -H 'Accept-Language: en-US,en;q=0.5' --compressed -H 'Referer: http://labs.consequentialrobotics.com/download.php?file=mdk_2-200131.tgz' -H 'Content-Type: application/x-www-form-urlencoded' -H 'Origin: http://labs.consequentialrobotics.com' -H 'DNT: 1' -H 'Connection: keep-alive' -H 'Upgrade-Insecure-Requests: 1' --data 'name=QUT&org=QUT&email=&agree=on' --output mdk.tgz \
&& tar -xzf mdk.tgz

RUN sed -i 's/MIRO_ROS_RELEASE=kinetic/MIRO_ROS_RELEASE=melodic/g' mdk-200131/share/config/user_setup.bash

RUN cd mdk-200131/catkin_ws/ \
    && rm -r install/ \
    && rm makefile \
    && source /opt/ros/melodic/setup.bash \
    && catkin config --install --extend /opt/ros/melodic \
    && catkin build

RUN cd mdk-200131/bin/deb64 \
    && mv libmiro_gazebo.so libmiro_gazebo7.so \
    && mv libmiro_gazebo9.so libmiro_gazebo.so \
    && ./install_mdk.sh