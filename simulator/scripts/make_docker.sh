#!/bin/bash

docker start $1

# Copy docker folder (need to be sudo)
dock_id=`docker inspect -f '{{.Id}}' $1`
srcpath=/var/lib/docker/aufs/mnt/$dock_id/sitl/catkin_ws/src
cp -r ~/KTH/SummerProject/catkin_px4_2/src/docker $srcpath/

# Make it
docker exec -it $1 'sh -c "cd /sitl/catkin_ws; catkin_make"'

docker stop $1