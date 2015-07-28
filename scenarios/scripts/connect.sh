#!/bin/bash


echo "ssh -t pi@$1 ./mavros_launch.sh $1 $2 $3"
gnome-terminal -e "ssh -t pi@$1 ./mavros_launch.sh $1 $2 $3"