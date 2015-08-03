#!/bin/bash

## Start the Mavros node
## this script is supposed to be called through a ssh command by the ROS master node

## Usage:
## ./mavros_launch.sh irisX ip_or_hostname_of_the_RPi master_uri
## irisX is the ns argument that will be used to launch mavros
## ip_or_hostname_of_the_RPi
## master_uri

# kill other nodes of mavros
sudo pkill mavros_node

# Output some text to be sure that the connection is working
figlet "Firing "$1
figlet "3..."
figlet "2..."
figlet "1..."

# source the indigo ROS environment
cd ~
. /opt/ros/indigo/setup.bash

# setup the ROS-Network variables
ROS_IP=$2
ROS_HOSTNAME=$2
ROS_MASTER_URI=$3

# Launch the node
roslaunch mavros connect.launch ns:=$1
