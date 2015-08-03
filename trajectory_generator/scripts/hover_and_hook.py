#!/usr/bin/env python
import rospy
import sys
import ast
import math
import controller
from trajectory import Trajectory
from controller.msg import Permission
from mocap.msg import QuadPositionDerived
from trajectory_generato import TrajectoryGenerator
from Trajectory_node import TrajectoryNode
from controller.srv import SetChannel6



class Hooker(Trajectory):
  """This class is used to hook an object while hovering at hovering_point.
  It is used in hook_demo.py."""
  
  ##@param done: tells whether or not the trajectory is done
  done = False
 
  def __init__(self,trajectory_node):
    Trajectory.__init__(self,trajectory_node)
    self.__node = trajectory_node
    self.tg = TrajectoryGenerator()


  ##@param hovering_point: the point at which the drone is hovering while using the hook
  def hover_and_hook(self, hovering_point):
    """This method is used to hook."""
    hovering_point.append(0.) #yaw
    msg = self.tg.get_message(hovering_point, [0.,0.,0.,0.], [0.,0.,0.,0.]) 
    r = 10.
    rate = rospy.Rate(r)
    self.__node.send_msg(msg)
    rate.sleep()
    rospy.sleep(3.)
    hook_func = rospy.ServiceProxy('blender/set_channel6', SetChannel6)
    hook_func(2000)
    self.__node.send_msg(msg)
    rate.sleep()

  ##@param hovering_point: the point at which the drone is hovering while using the hook
  def hover_and_unhook(self, hovering_point):
    """This method is used to release the load."""
    hovering_point.append(0.) #yaw
    msg = self.tg.get_message(hovering_point, [0.,0.,0.,0.], [0.,0.,0.,0.]) 
    r = 10.
    rate = rospy.Rate(r)
    self.__node.send_msg(msg)
    rate.sleep()
    rospy.sleep(3.)
    hook_func = rospy.ServiceProxy('blender/set_channel6', SetChannel6)
    hook_func(0)
    self.__node.send_msg(msg)
    rate.sleep()
    
  def begin(self):
    pass

  def loop(self,start_time):
    pass
    

  




  
