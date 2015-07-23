#!/usr/bin/env python

# This script creates a node and publishes trajectory points on the topic
# trajectory_gen/target. 


import rospy
from mocap.msg import QuadPositionDerived
from controller.msg import Permission
from trajectory_generato import TrajectoryGenerator
import os
from trajectory import Trajectory
import math



class TrajectoryNode():
  """This class provides the trajectorynode to be used by every object
  that inherits from trajectory."""  
  
  pub = None
  
  def __init__(self,group=''):
    """The node is initialized and a publisher created.
    The messages are published on the correct topic for the 
    controller."""
    rospy.init_node('TG')
    abspath = ""
    if group:
    	abspath = "/"+group
    self.__pub = rospy.Publisher(abspath+'trajectory_gen/target',QuadPositionDerived, queue_size=10)
    self.__security_pub = rospy.Publisher(abspath+'trajectory_gen/done', Permission, queue_size=10)
    
  # Publishes messenges on the topic trajectory_gen/target. The type of the
  # message should be QuadPositionDerived. 
  def send_msg(self, msg):
    """This method is used to publish the message msg."""
    self.__pub.publish(msg)


  # Publishes a boolean on the topic trajectory_gen/done. This should be used
  # inform subscribers that the trajectory is done.
  def send_permission(self, boolean):
    """This method is used to publish True or False on
    on the topic that let's the security guard know
    that the trajectory is done."""
    self.__security_pub.publish(Permission(boolean))
    
    
    

