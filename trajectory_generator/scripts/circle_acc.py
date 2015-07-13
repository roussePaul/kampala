#!/usr/bin/env python
import rospy
import sys
import ast
import math
import numpy
from mocap.msg import QuadPositionDerived
from trajectory_generato import TrajectoryGenerator
from trajectory import Trajectory
from Trajectory_node import TrajectoryNode
from straight_line_class import StraightLineGen

#This script generates the points, velocities and accelerations to be used as a reference for the 
#controller to to get the quad to accelerate into a circle.
#Given a midpoint, starting point and target speed, the quad will accelerate up to this speed #tracking an arc.
#The angle as a function of time is given by theta = v*t^2/(2*t_f*R), where v is the target speed, #R the radius of the circle and t_f the time the acceleration will take, calculated from #constraints on the acceleration. 
#It is rather similar to arc.py.

class AccGen(Trajectory):
  
  done = False
  a_max = 0.6**2.0/0.8
  t_f = 0
  
  def __init__(self,trajectory_node,mid,start,velo):
    Trajectory.__init__(self,trajectory_node)
    self.tg = TrajectoryGenerator()
    self.midpoint = mid
    self.start = start
    n = [self.start[0]-self.midpoint[0],self.start[1]-self.midpoint[1],self.start[0]-self.midpoint[2]]
    self.radius = self.tg.get_distance(self.start,self.midpoint)
    self.initial_velo = velo
    self.velo = self.tg.get_norm(self.initial_velo)
    #define new coordinates
    self.e_n = self.tg.get_direction(n)
    self.yp = self.tg.get_direction(self.initial_velo)
    self.zp = numpy.cross(self.e_n,self.yp)
    self.w = self.radius*self.velo
    self.theta_z = self.tg.get_projection([0,0,1],self.e_n)
  
  def begin(self):
    v_max = (self.radius*self.a_max)**(0.5)
    if self.velo > v_max:
      self.velo = v_max
    self.__set_done(False)
    

  def loop(self, start_time):
    self.__set_t_f(self.velo/math.sqrt(self.a_max**2.0 - self.velo**4.0/self.radius**2.0))
    time = start_time
    r = 10.0
    rate = rospy.Rate(r)
    while not rospy.is_shutdown() and not is_done():
      theta = self.velo*time**2.0/(2*self.radius*t_f)
      w = self.velo*time/(self.radius*t_f)
      alpha = self.velo/(self.radius*t_f)
      outpos = self.tg.get_circle_point(self.radius, theta)
      outpos = self.tg.offset(outpos,self.midpoint)
      outpos.append(self.tg.adjust_yaw([1,0,0]))
      outvelo = self.tg.get_circle_velocity(self.radius,theta,omega)  
      outvelo.append(0)
      outacc = self.tg.get_circle_acc(self.radius,theta,omega,alpha)
      outacc.append(0)
      outmsg = self.tg.get_message(outpos,outvelo,outacc)
      self.trajectory_node.send_msg(outmsg)
      self.trajectory_node.send_permission(False)
      rate.sleep()
      time += 1/r
      if time >= t_f:
        self.__set_done(True)
      

    
  def is_done(self):
    return self.done

  def __set_done(self,boolean):
    self.done = boolean

  def __set_t_f(self,t):
    self.t_f = t

  def get_t_f(self):
    return self.t_f
   

