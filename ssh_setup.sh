#!/bin/sh

rm -r .ssh
ssh -oHostKeyAlgorithms='ssh-rsa' pi@192.168.0.11 echo ""
ssh-keygen -t rsa
ssh pi@192.168.0.11 mkdir -p .ssh
cat .ssh/id_rsa.pub | ssh pi@192.168.0.11 'cat >> .ssh/authorized_keys'
ssh pi@192.168.0.11

# dns things deals with the router wich affect the hostname of the rpi
# every computer must have right IP
# Currently implemented in bashrc
# To properly switch off the quad, you are supposed to shutdown the pi with the "sudo shutdown 0" command.
# pulling down the current can corrupt some files, then one day you will probably have to reinstall the pi system on the SD card (it is not a big deal)