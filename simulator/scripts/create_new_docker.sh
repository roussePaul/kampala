#!/bin/sh
# script to create a new docker for the simulated drone
# usage:
# 	create_new_docker.sh docker-iris-1


WDIR=`pwd`


sudo docker run --privileged -e DISPLAY=$DISPLAY --name=$1 -it px4io/px4-ros-full bash 

docker start $1
docker exec -it $1 bash -c "`cat init_ws_docker.sh`"
