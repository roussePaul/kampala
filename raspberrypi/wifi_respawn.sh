#!/bin/bash

## Script to keep the network up on the RPi
## This script will be only usefull if the network went down or if it was not up at the boot of the RPi.


# If the script is already running, don't run it again
echo "Launching wifi respawn program"

# while the network is not up, try to reconnect to it
while true; do
   ifup  wlan0
   sleep 10
done
