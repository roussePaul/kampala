#!/usr/bin/env python



import rospy
import sys
import ast
import math
import numpy
from mocap.msg import QuadPositionDerived
from trajectory_generato import TrajectoryGenerator
from trajectory import Trajectory
from trajectory_node import TrajectoryNode
from straight_line_class import StraightLineGen


class AccGen(Trajectory):
  """This script generates the points, velocities and accelerations to be used as a reference for the 
  controller to to get the quad to accelerate into a circle.
  Given a midpoint, starting point and target speed, the quad will accelerate up to this speed tracking an arc.
  The angle as a function of time is given by theta = v*t^2/(2*t_f*R), where v is the target speed, 
  R the radius of the circle and t_f the time the acceleration will take, calculated from 
  constraints on the acceleration. 
  It is rather similar to arc.py."""
  
  ##@param done: tells whether or not the trajectory is done 
  done = False
  ##@param a_max: the maximum possible acceleration
  a_max = 0.6**2.0/0.8
  ##@param t_f: the time the acceleration phase will take, to be set later
  t_f = 0
  
  def __init__(self,trajectory_node,mid,start,velo):
    Trajectory.__init__(self,trajectory_node)
    self.tg = TrajectoryGenerator()
    self.__midpoint = mid
    self.__start = start
    n = [self.__start[0]-self.__midpoint[0],self.__start[1]-self.__midpoint[1],self.__start[2]-self.__midpoint[2]]
    self.__radius = self.tg.get_distance(self.__start,self.__midpoint)
    self.__velo = self.tg.get_norm(velo)
    #define new coordinates
    xp = self.tg.get_direction(n)
    yp = self.tg.get_direction(velo)
    self.__tensor = [xp,yp]
  
  def begin(self):
    v_max = (self.__radius*self.a_max)**(0.5)
    if self.__velo > v_max:
      self.__velo = v_max
    self.__set_done(False)
    
  ##@param start_time: sets the initial time used for generating the circle
  def loop(self, start_time):
    """This method is called to perform the acceleration."""
    self.__set_t_f(self.__velo/math.sqrt(self.a_max**2.0 - self.__velo**4.0/self.__radius**2.0))
    time = start_time
    r = 10.0
    rate = rospy.Rate(r)
    while not rospy.is_shutdown() and not self.is_done():
      theta = self.__velo*time**2.0/(2*self.__radius*self.t_f)
      w = self.__velo*time/(self.__radius*self.t_f)
      alpha = self.__velo/(self.__radius*self.t_f)
      outpos = self.tg.get_circle_point(self.__radius, theta)
      outpos = self.__transform(outpos)
      outpos = self.tg.offset(outpos,self.__midpoint)
      outpos.append(self.tg.adjust_yaw([1,0,0]))
      outvelo = self.tg.get_circle_velocity(self.__radius,theta,w)
      outvelo = self.__transform(outvelo)  
      outvelo.append(0)
      outacc = self.tg.get_circle_acc(self.__radius,theta,w,alpha)
      outacc = self.__transform(outacc)
      outacc.append(0)
      outmsg = self.tg.get_message(outpos,outvelo,outacc)
      self.trajectory_node.send_msg(outmsg)
      self.trajectory_node.send_permission(False)
      rate.sleep()
      time += 1/r
      if time >= self.t_f:
        self.__set_done(True)
      
  ##@param vec: the vector to be transformed
  ##@return the transformed vector
  def __transform(self,vec):
    """This function transforms a vector that is given in the coordinate frame, which has an
    x-axis defined by the vector from the midpoint to the startpoint and a y-axis defined by 
    the initial velocity vector (local frame), to the sml-frame. It is assumed that the vector is of the form
    [x,y,0] as all points on the circle are in the xy-plane in the local frame."""
    ret_vec = [0.,0.,0.]
    for i in range(0,3):
      for j in range(0,2):
        ret_vec[i] += vec[j] * self.__tensor[j][i]
    return ret_vec 
    
  def is_done(self):
    return self.done

  def __set_done(self,boolean):
    self.done = boolean

  def __set_t_f(self,t):
    self.t_f = t

  ##@return the time the acceleration phase lasts
  def get_t_f(self):
    return self.t_f
   

