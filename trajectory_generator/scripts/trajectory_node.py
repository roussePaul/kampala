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
  
  pub = None
  
  def __init__(self,group=''):
    rospy.init_node('TG')
    abspath = ""
    if group:
    	abspath = "/"+group
    self.pub = rospy.Publisher(abspath+'trajectory_gen/target',QuadPositionDerived, queue_size=10)
    self.security_pub = rospy.Publisher(abspath+'trajectory_gen/done', Permission, queue_size=10)
    
  # Publishes messenges on the topic trajectory_gen/target. The type of the
  # message should be QuadPositionDerived. 
  def send_msg(self, msg):
    self.pub.publish(msg)


  # Publishes a boolean on the topic trajectory_gen/done. This should be used
  # inform subscribers that the trajectory is done.
  def send_permission(self, boolean):
    self.security_pub.publish(Permission(boolean))
    
    
    

