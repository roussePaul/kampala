#!/bin/sh
# script to create a new docker for the simulated drone
# usage:
# 	create_new_docker.sh docker-iris-1

docker run --privileged -e DISPLAY=$DISPLAY --name=docker-iris-1 -it px4io/px4-ros-full bash 
docker docker exec -it docker-iris-1 bash -c "`cat init_ws_docker.sh`"
sudo docker run --privileged -e DISPLAY=$DISPLAY --name=docker-iris-2 -it px4io/px4-ros-full bash -c "`cat init_ws_docker.sh`"
