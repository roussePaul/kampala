#!/usr/bin/env python
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
    self.pub = rospy.Publisher(abspath+'trajectory_gen/target',QuadPositionDerived, queue_size=10)
    self.security_pub = rospy.Publisher(abspath+'trajectory_gen/done', Permission, queue_size=10)
    
  def send_msg(self, msg):
    """This method is used to publish the message msg."""
    self.pub.publish(msg)

  def send_permission(self, boolean):
    """This method is used to publish True or False on
    on the topic that let's the security guard know
    that the trajectory is done."""
    self.security_pub.publish(Permission(boolean))
    
    
    

