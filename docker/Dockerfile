# krock-sim
FROM  osrf/ros:kinetic-desktop-full-xenial

ENV DEBIAN_FRONTEND noninteractive

##
# ROS base
##
RUN apt-get -y update && apt-get install -y ros-kinetic-desktop-full ros-kinetic-joystick-drivers ros-kinetic-ros-control
CMD source /opt/ros/kinetic/setup.bash

##
# Extra packages
##
RUN apt-get -y update && apt-get install -y libarmadillo-dev curl git python-catkin-tools dbus python-pip gfortran nano packagekit-gtk3-module libcanberra-gtk-module gedit eog gdebi nautilus python-pip python-dev build-essential libpulse-mainloop-glib0

RUN apt-get -y update && apt-get install -y gdb htop

##
# webots and other tools and sources

COPY src /root/src

RUN apt-get -y update && yes | gdebi /root/src/webots_2018b_amd64.deb

##
# For gpu usage (specially nvidia gpu).
##
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

##
# ros workspace setup
##
RUN  /bin/bash -c "mkdir -p ~/catkin_ws/src ; source /opt/ros/kinetic/setup.bash ; cd ~/catkin_ws/src ; source /opt/ros/kinetic/setup.bash ; catkin_init_workspace"

##
# command
##
WORKDIR /root/catkin_ws
CMD bash

##
# Executing with gpu support
##
#
# Run nvidia-docker deamon
# sudo -b nohup nvidia-docker-plugin > /tmp/nvidia-docker.log
#
#xhost  +
#
# run container using nvidia-docker-compose run krock-sim
