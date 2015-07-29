#!/bin/bash

## Script to keep the network up on the RPi
## This script will be only usefull if the network went down or if it was not up at the boot of the RPi.


# If the script is already running, don't run it again
var=`ps -A | grep wifi_respawn | wc -l`
if [ $var -ge 2 ]; then
   exit
fi

# while the network is not up, try to reconnect to it
while !(ifconfig wlan0 | grep -q "inet addr:") ; do
   echo "Network connection down! Attempting reconnection."
   ifup  wlan0
   sleep 10
done
