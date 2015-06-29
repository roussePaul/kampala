#!/usr/bin/env python
import rospy
from mocap.msg import QuadPositionDerived
from controller.msg import Permission
from trajectory_generato import TrajectoryGenerator
import os
from trajectory import Trajectory
import math

class TrajectoryNode():
  
  pub = None
  
  def __init__(self):
    rospy.init_node('TG')
    self.pub = rospy.Publisher('trajectory_gen/target',QuadPositionDerived, queue_size=10)
    self.security_pub = rospy.Publisher('trajectory_gen/done', Permission, queue_size=10)
    
  def send_msg(self, msg):
    self.pub.publish(msg)

  def send_permission(self, boolean):
    self.security_pub.publish(Permission(boolean))
    
    
    

