#!/bin/bash


echo "[DOCKER] START: $1"

docker stop $1
docker start $1

gnome-terminal -x sh -c "docker exec -it $1 /sitl/catkin_ws/src/Kampala/simulator/launch/init_iris.sh $2 $3 $4;read -p \"Press any key to continue...\" a;"