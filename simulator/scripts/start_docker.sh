#!/bin/bash

## Used to start the container of the iris px4 firmware

echo "[DOCKER] START: $1"

docker stop $1
docker start $1

gnome-terminal -x sh -c "docker exec -it $1 /sitl/catkin_ws/src/kampala/simulator/launch/init_iris.sh $2 $3 $4;read -p \"Press any key to continue...\" a;"