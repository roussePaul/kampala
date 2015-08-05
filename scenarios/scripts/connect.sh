#!/bin/bash


echo "ssh -t pi@$1 \"./mavros_launch.sh $1 $2 $3\";read"
gnome-terminal -x bash -c "ssh -t pi@$1 './mavros_launch.sh $1 $2 $3';read"
