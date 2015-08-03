# INSTALLATION GUIDE OF THE RASPBERRY PI

## HARDWARE

### How does it work:

* Power supply:
The raspberry pi get the power from a BEC converter (provide a stable voltage of about 5V).
The BEC does not really provide 5V, it is more around 5.3V which is higher than what the Raspberry Pi is designed for (a usb port provide 5V +/- 25%). However it seems to work like this, and according to some forums, the critical voltage is more about 5.6V of 6V. The BEC take the power from the battery through the Gimbal connector.
We have mounted some connetors on the BEC and on the USB cable. The one on the USB cable is unuseful, you can dismiss it for the next time.

* How to choose a BEC converter:
You have to find one that will provide a clean 5V signal and as close as possible from 5V.
Current needed by the RPi does not exceed 2A, so a 3A BEC is enough.
This one has been successfully used for the pi during some hours: http://www.hobbyking.com/hobbyking/store/__22494__Turnigy_3A_UBEC_with_Low_Voltage_Buzzer.html
It is also possible to use the BEC of an ESC. 

* Wifi:
The RPi is connected to the network thanks to a wifi sticks.
It is always possible to connect it to the network with an ethernet cable.

* Mavlink:
Direct connection to the quad with a USB cable. It is possible to use one of the internal connection of the quad (TELEM1 and TELEM2). See Mission Planner to change the baudrate.

* Router:
We will use our own router in order to connect to the quadcopter so that the link between our quads and the computers is more direct. The router needs to have wifi, some ethernet ports, a name resolving algorithm (I don't really know the name of this feature, the idea is that we need to be able to do a ping A between machine with hostname A and hostname B).


## SOFTWARE

### Raspberry Pi
You have 2 options, either duplicate a working system and adapting some files to get the raspberry pi working on another quad.
Or to reinstall everything. The second option will take you 10 hours more or less (the RPi is pretty slow for compiling ROS), the first one 1 hour.

#### 1st option
* Reinstall everything:
Download Rasbian OS: https://www.raspberrypi.org/downloads/
Install it with this tutorial: https://www.raspberrypi.org/documentation/installation/installing-images/linux.md
Connect the RPi to the router by ethernet, try a ssh pi@raspberrypi, if it does not work, try to locate the RPi: nmap -sn THE_NETWORK_OF_THE_ROUTER
Setup the wifi connection/internet connection.
Install ROS: http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi (See the troubleshoot section)
Install Mavros: (see the 3. of the previous link with "mavros" package)
Install every files of this directory to the home directory.
Do the rest

* Wifi setup
To setup the connection, you have to deal with these files
```Bash
/etc/network/interface
/etc/wpa_supplicant/wpa_supplicant.conf
```
In order to make the RPi connect automatically to the network, a little script is executed every 5 min to see if the connection is up. It is using the cron table, to change the configuration, look at:
```Bash
~/wifi_respawn.sh
```

* ROS mavros
copy the connect.launch in the mavros/launch directory (roscd mavros/launch)

#### 2sd option
* Backup an existing RPi 
```Bash
dd if=/dev/FIND_THE_SD_CARD_OF_THE_RPI of=sd.img bs=4M
```

* Restore from a backup
```Bash
dd if=sd.img of=/dev/FIND_THE_SD_CARD_OF_THE_RPI bs=4M
```

* Network
change the /etc/hostname of the RPi with the iris1, iris2 or ...
reboot the rpi

* SSH setup
We are using automatic connection to the RPi. You need to configure your computer so that it works.
Before you do anything, try to understand what SSH is: https://www.digitalocean.com/community/tutorials/how-to-set-up-ssh-keys--2
(don't do the tutorial).
ROS does not support the ECDSA login, if you just connect to the RPi with the ssh command, it will initialize the connection with this algorithm. To avoid that, connect to the RPi with the 
```Bash
ssh -oHostKeyAlgorithms='ssh-rsa' pi@IRIS_HOSTNAME_HERE echo ""
```
command. If you have already been connected to the RPi, delete the correponding line in the ~/.ssh/known_hosts file (I don't know how to find the right line).
Generate the key with
```Bash
ssh-keygen -t rsa
```
and copy it to the RPi like this
```Bash
ssh pi@IRIS_HOSTNAME_HERE mkdir -p .ssh
cat .ssh/id_rsa.pub | ssh pi@IRIS_HOSTNAME_HERE 'cat >> .ssh/authorized_keys'
```
Then if you can connect to the RPi without your password, that is really good 
```Bash
ssh pi@IRIS_HOSTNAME_HERE
```
If not try to figure out why (I used these websites http://www.linuxproblem.org/art_9.html https://www.digitalocean.com/community/tutorials/how-to-set-up-ssh-keys--2)

#### ROS
To understand how ROS is setup with several machines, look at:

http://wiki.ros.org/ROS/Tutorials/MultipleMachines

http://wiki.ros.org/ROS/NetworkSetup

* ROS-Network configuration of the computer:
3 environment variables are dealing with this:
```Bash
ROS_IP
ROS_HOSTNAME
ROS_MASTER_URI
```
you can see them with:
```Bash
export | grep ROS
```
These are setup in the ~/.bashrc file with the IP of the computer (so that it does work even if the hostname is unknown).
The install.sh take care about adding this to the ~/.bashrc file.

* ROS-Network configuration of the RPi:
the mavros_launch.sh script is setting up these variable at the beggining according to the argument it received.

## What happen if...

* The network connection go down for a while?
Mavros will be killed and the quad will notice it after 2 or 5 seconds, it will go in Landing mode.
The wifi network will began to be restored within 10 seconds. 
You will probably have a strange behavior for the next start of the quadcopter!, be prepared, otherwise you can just switch off every thing and restart it again.

* I arm the quad before the parameter initialization is over?
The quad will randomly go to a direction and land a while after.

## Troubleshoot

* Sometimes it is not possible to connect the RPi to the Quad with Mavlink. Try with the "mavproxy.py" command. If it does not work, switch off the quad and the Pi and restart it.
Maybe this come from perturbations of the BEC (I don't really know how much a BEC can disturb Pixhawks).

* Mavros is able to connect to the quad even if you don't give him any parameters (with the ros param fcu_url=""). However, the baudrate chosen by mavros does not correspond to the one that is configured in Mission Planner. This is weird, but it is working like this, so we will leave it as it is. 

* Mocap connection does not find the quad anymore! The force the security gard to land the quad. Is there any collision between the RPi connection and the Mocap one?

* Boost error:

```Bash
terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
what():  boost::lock_error
================================================================================REQUIRED process [iris2/mavros-1] has died!
process has died [pid 3521, exit code -6, cmd /opt/ros/indigo/lib/mavros/mavros_node /mavlink/to:=mavlink/to /mavlink/from:=mavlink/from __name:=mavros __log:=/home/pi/.ros/log/9af0a020-35ce-11e5-b397-b8ca3a8a3325/iris2-mavros-1.log].
log file: /home/pi/.ros/log/9af0a020-35ce-11e5-b397-b8ca3a8a3325/iris2-mavros-1*.log
Initiating shutdown!
================================================================================
[iris2/mavros-1] killing on exit
```
Try first to reboot the Rpi.

 With the standard installation of ROS on the RPi, sometimes the RPi stop the connection during the flight.
Apparently, it is coming from a boost version error, ROS seems to works better with the 1.46.1 version of boost (http://answers.ros.org/question/64191/arm-boost-exception-what-boostlock_error/)
How to install the right boost library?
* Not tested but should work:
Do a regular installation of ROS on the raspberry pi on: http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi
Remove all boost libraries and install the 1.46 version: 
```Bash
sudo apt-get remove libboost*
sudo apt-get install libboost1.46-dev libboost-thread1.46-dev libboost-system1.46-dev libboost-test1.46 libboost-test1.46-dev libboost-filesystem1.46-dev libboost1.46-all-dev 
```
reinstall console_bridge (remove every existing files before )
```Bash
cd ~/ros_catkin_ws/external_src
rm REMOVE THE FILES OF LIB CONSOLE HERE
git clone https://github.com/ros/console_bridge
cd console_bridge
cmake .
!! sudo checkinstall make install !! --> When check-install asks for any changes, the name (2) needs to change from "console-bridge" to "libconsole-bridge-dev" otherwise the rosdep install wont find it. 
```
create symbolic links from libboost 1.46.1 to 1.49 so that ros find them

* Otherwise, follow the installation tutorial of ROS: http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi
during the installation of the dependencies each time simulate the apt-get before running it (add the option -s to the apt-get command), then check if there is any libboost things that are installed, if so find the 1.46 version)
install the console bridge in this way
```Bash
cd ~/ros_catkin_ws/external_src
git clone https://github.com/ros/console_bridge
cd console_bridge
cmake .
!! sudo checkinstall make install !! --> When check-install asks for any changes, the name (2) needs to change from "console-bridge" to "libconsole-bridge-dev" otherwise the rosdep install wont find it. 
```
