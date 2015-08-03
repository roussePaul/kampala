# [Kampala](https://en.wikipedia.org/wiki/Kampala)



## ROS installation

Execute the following script:

```Bash
set -e
export DEBIAN_FRONTEND=noninteractive
 
# Main ROS Setup
# Following http://wiki.ros.org/indigo/Installation/Ubuntu
# Also adding dependencies for gazebo http://gazebosim.org/tutorials?tut=drcsim_install
 
## add ROS repository and key
## install main ROS pacakges
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get -y install ros-indigo-desktop-full
sudo rosdep init
rosdep update
 
## setup environment variables
sudo sh -c 'echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc'
source ~/.bashrc
 
## get rosinstall and some additional dependencies
sudo apt-get -y install python-rosinstall ros-indigo-octomap-msgs ros-indigo-joy ros-indigo-geodesy
 
## add osrf repository
## install drcsim
sudo sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu trusty main" > /etc/apt/sources.list.d/drc-latest.list'
wget http://packages.osrfoundation.org/drc.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get -y install drcsim
 
## install mavros but from shadow repo to get latest version earlier
sudo sh -c 'echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu/ trusty main" > /etc/apt/sources.list.d/ros-shadow.list'
sudo apt-get update
sudo apt-get install ros-indigo-mavros ros-indigo-mavros-extras
```

## Workspace setup

```Bash
set -e
 
WDIR=`pwd`
WORKSPACE=$WDIR/catkin_ws
 
# Setup workspace
mkdir -p $WORKSPACE/src
cd $WORKSPACE/src
catkin_init_workspace
cd $WORKSPACE
catkin_make
sh -c "echo 'source $WORKSPACE/devel/setup.bash' >> ~/.bashrc"
 
# Fetch sources
## PX4 firmware
cd $WORKSPACE/src
git clone https://github.com/PX4/Firmware.git
 
## rotors simulator
cd $WORKSPACE/src
git clone https://github.com/PX4/rotors_simulator.git
 
## mav comm
cd $WORKSPACE/src
git clone https://github.com/PX4/mav_comm.git
 
## glog catkin
cd $WORKSPACE/src
git clone https://github.com/ethz-asl/glog_catkin.git
 
## catkin simple
cd $WORKSPACE/src
git clone https://github.com/catkin/catkin_simple.git
 
## Kampala
cd $WORKSPACE/src
git clone https://github.com/roussePaul/kampala.git
 
 
# Disable parallel make jobs for compilation.
# Sometimes required inside Docker container or VMs with not enough memory.
# If you're on a native Ubuntu installation, you can leave this away.
export ROS_PARALLEL_JOBS=
 
# Compile workspace
cd $WORKSPACE
source devel/setup.bash
catkin_make

echo "ip=\`/sbin/ifconfig eth0 | grep 'inet addr:' | cut -d: -f2 | awk '{ print \$1}'\`
export ROS_IP=\$ip
export ROS_HOSTNAME=\$ip
export ROS_MASTER_URI=http://\$ip:11311" >> ~/.bashrc
```

## Python packages
Install the following packages:

dxfwrite `sudo pip install dxfwrite`

dxfgrabber `sudo pip install dxfgrabber`

scipy `http://www.scipy.org/install.html`

control `sudo pip install control`

controller `sudo pip install controller`

gnosis `sudo pip install gnosis`

matplotlib `sudo apt-get install python-matplotlib`

yaml `pip install pyyaml`

# Connection Setup
You have 2 ways of communicate with the quadcopter:

## 3DR Radio
Follow the guide in [scenarios/launch/iris/INSTALL](scenarios/launch/iris/INSTALL).

## Raspberry pi
Follow the guide in the [raspberrypi](raspberrypi).

# Simulator Setup
Follow the guide in the [simulator](simulator).