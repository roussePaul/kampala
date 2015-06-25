#!/bin/bash

source /sitl/catkin_ws/devel/setup.bash


set -x

export ROS_IP=`ifconfig | grep -Eo 'inet (addr:)?172\.([0-9]*\.){2}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'`
export ROS_HOSTNAME=$ROS_IP
export ROS_MASTER_URI=http://172.17.42.1:11311

roslaunch simulator docker.launch ns:=$1 x:=$2 y:=$3
