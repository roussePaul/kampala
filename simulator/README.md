Simulator package
=================

Contains every files needed to run the simulator.

# Goal
Simulate the Iris+
Simulate additionnal components (grippers, pay load, ...)
Same mavlink interface
Same controller

# Implementation
The [Explored Solutions](#explored-solutions) gives more informations about the implementation.

## Differencies

The controller used in APM is the STABILIZE one. You control roll and pitch positions, yaw rate and the motor speeds. The one used in the simulation have the same inputs
However, we noticed that the maximum tilting angle of the quad is smaller than the real quad (real quad: 45 degrees, simulation+PX4: 10 degrees).

Mavros is partly available. Since the interface protocol is slightly different between the PX4 firmware and the APM one, it was not possible to connect directly the simulated mavlink interface to the one mavros node.
More over the mavlink connection is probably faster than the reallity.

There is a strange behavior of Gazebo, the positions given by gazebo are noisy even if the URDF file does not s
pecify any noise on the odometry sensors! This is even stranger because the link_states topic is more noisy than the model_states one.
To fix that we have been filtering the outputs of Gazebo. To change the cutoff frequency of the filters, you can edit the files in the ros_mocap_sim.py file.

# Installation

## PX4 SITL

Follow this tutorial: https://pixhawk.org/dev/ros/sitl

To install the simulated quads, first, you need to follow: https://pixhawk.org/dev/ros/automated_sitl. Perform the step so that we are able to use docker as a not root user.

Then follow the next section to install the simulated quads.

## Add a new quadcopter

Execute the following commands:
```Bash
cd ~/catkin_ws/src/kampala/simulation/scripts
docker run --privileged -e DISPLAY=$DISPLAY --name=docker-iris-1 -it px4io/px4-ros-full bash 
docker start docker-iris-1
docker exec -it docker-iris-1 bash -c "`cat init_ws_docker.sh`"
```
Change the docker-iris-1 to the iris you want to create (ex: docker-iris-2).
The configuration of the simulated quad is made in the scenarios/launch/iris/irisX.launch file: 
```XML
<!-- Body id of the quad that will be used by the mocap simulation script -->
<arg name="body_id" value="2"/>
<!-- Docker name of the sitl simulation -->
<arg name="docker" value="docker-iris-2"/>
```

# Gazebo

## URDF description
Small introduction to the URDF syntax: http://gazebosim.org/tutorials/?tut=ros_urdf. As we are using an old Gazebo version, the URDF tutorial is not fully supported.
The files used to for the iris description are in the [description](description/) folder.
They are using a macro language that enable us to use some variables inside ROS: [Xacro](http://wiki.ros.org/xacro).

## Add a component to the quad

We did not try to add an external component to the quad directly through a URDF description. However, it should be possible.

Instead we have directly been added the components inside the [iris_base.xacro](description/iris_base.xacro) file.

## Track the position with mocap
To get the position, of a link, you can add:
```XML
<xacro:odometry_plugin_macro
      namespace="${namespace}/load_pose"
      odometry_sensor_suffix="gt"
      parent_link="${namespace}/load_link"
      pose_topic="pose"
      pose_with_covariance_topic="pose_with_covariance"
      position_topic="position"
      transform_topic="transform"
      odometry_topic="odometry"
      parent_frame_id="world"
      mass_odometry_sensor="0.00001"
      measurement_divisor="1"
      measurement_delay="0"
      unknown_delay="0.0"
      noise_normal_position="0 0 0"
      noise_normal_quaternion="0 0 0"
      noise_normal_linear_velocity="0 0 0"
      noise_normal_angular_velocity="0 0 0"
      noise_uniform_position="0 0 0"
      noise_uniform_quaternion="0 0 0"
      noise_uniform_linear_velocity="0 0 0"
      noise_uniform_angular_velocity="0 0 0"
      enable_odometry_map="false"
      odometry_map=""
      image_scale=""
    >
      <inertia ixx="0.00001" ixy="0.0" ixz="0.0" iyy="0.00001" iyz="0.0" izz="0.00001" /> <!-- [kg.m^2] [kg.m^2] [kg.m^2] [kg.m^2] [kg.m^2] [kg.m^2] -->
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
</xacro:odometry_plugin_macro>
```
inside the URDF file.

To add it to the tracked body inside mocap you have to change the [mocap launch file](../mocap/launch/mocap.launch). Read the comments to figur out how to do it.

# Explored solutions

Several approaches has been explored to setup the simulator. Here is a summary in the chronological order of what I have done about it.
This will provide you reasons about the choosen solutions and also an insight of other solutions that can be settle and what kind of difficulties that have been met during there setup.

## RotorS library

The [RotorS library](https://github.com/ethz-asl/rotors_simulator) provide few quadcopters with 2 basic ontrollers. They are usable within ROS and Gazebo.
However, as the model of Iris was available inside the PX4 SITL package, and as it was not straight forward to adapt the controller to the Iris model (sensor frames differents, need to tune the PID gains), we decided to give a try to the PX4 SITL one. One reason was also to have a behaviour to the real quadcopter.

### References
RotorS library: https://github.com/ethz-asl/rotors_simulator

## APM SITL

As our quadcopters were using APM, we first tried to find a simulator that is close to the APM firmware.
APM provide a Arducopter firmware that can run on a regular computer: the APM SITL Simulator (SITL stamd for Software in the Loop).
Then what happen inside the Pixhawk hardware can be reproduce exactly in the same way except for the sensor information that are no longer provided by regular sensor on the board but by simulated one that you can control.

It is possible to modify the simulator that is provided with the standard SITL code and to make it communicate with Gazebo through ROS, 

The [architecture](http://dev.ardupilot.com/wiki/simulation-2/sitl-simulator-software-in-the-loop/) of the APM SITL implementation is made so that you can use different simulator with it. All you have to do is to connect your simulator through the UDP ports 5502 (PWM outputs of the controller) and the 5501 (input sensor data produced by the simulator).

We successfully managed to setup the communication between Gazebo and the APM SITL software by creating an interface that can transform every messages from ROS to UDP and from UDP to ROS.

However, the sensor frames that are used inside RotorS and the ones inside the SITL software are not the same (if I remember well, every sensors was turned of about a quarter circle).
In the real situation, Mission Planner automatically setup frame orientations during the accelerometer and the magnetometer initialization. 
To setup the differents frames, we have been doing step responses along axes and comparing them to the given simulator so that it gives the same results.
Unfortunatly, even after having the right sensor orientation, the quad was still not stabilizable! For an unknown reason, the quad was not behaving in the same way as in the given SITL simulator.


### References

Architecture of the SITL implementation on APM: http://dev.ardupilot.com/wiki/simulation-2/sitl-simulator-software-in-the-loop/

How to set up the APM SITL software on linux: http://dev.ardupilot.com/wiki/simulation-2/sitl-simulator-software-in-the-loop/setting-up-sitl-on-linux/


## PX4 SITL + RotorS

Like APM, the PX4 autopilot project provide a [SITL simulator](https://pixhawk.org/dev/ros/sitl) that can run within ROS using Gazebo and RotorS.
By default it is simulating the Iris+ quadcopter. However we don't know if the dynamical parameters has been measured or just set to some default values were working quite good with it.
It is really easy to setup 1 quadcopter and control it, you just have to follow the instructions on the [setup of the PX4 SITL](https://pixhawk.org/dev/ros/sitl). However it is not possible to launch severals quadcopters simulation at the same time as they are not only communicationg through ROS but also through regular network protocols (UDP, TCP).

Thankfully, ETH had anticipated it by setting up a docker containers that can run ROS and the PX4 firmware. Docker containers are very light virtual machines that take advantage of the already running kernel of your computer to run another linux without running 2 times the system core programs, you can find more informations about it on the [official website of Docker](https://www.docker.com/) or on the [wikipedia page](https://en.wikipedia.org/wiki/Docker_%28software%29). The docker will create a subnetwork that will be used be its localhost. It is therefore possible to comminicate with them using their network interface.

This require some more setup on ROS so that the packets can reach the nodes that are outside of the roscore network.
More information about it can be found on the [Network Setup](http://wiki.ros.org/ROS/NetworkSetup) page of the ROS wiki.



### References

https://pixhawk.org/dev/ros/sitl

https://pixhawk.org/dev/ros/automated_sitl

https://www.docker.com/

http://wiki.ros.org/ROS/NetworkSetup

## Choosen solution
We have choosen the PX4 SITL mainly because it was the first one we managed to make it work.

Advantages:
* as it is running the PX4 firmware, it is really possible to add an extra controller and test it on the simulator before trying it on the real quad.

Inconvenients:
* it is the not the APM firmware
* the setup is coöplex and not straight forward to do (need dockers)