#!/usr/bin/env python
import rospy
import sys
import ast
import math
from trajectory import Trajectory
from controller.msg import Permission
from mocap.msg import QuadPositionDerived
from trajectory_generato import TrajectoryGenerator
from Trajectory_node import TrajectoryNode
#Generates a straight line between startpoint and endpoint with velocity zero at the start and #endpoint.
#The time law is of the form s(t) = t^2*constant*(t-3/2*t_f). Here t_f is calculated so that the 
#speed at the end_point is zero.
#Respects constraints on acceleration.

class StraightLineGen(Trajectory):
  
  done = False
  a_max = 9.81/3.
 

  def __init__(self,trajectory_node,start,end):
    Trajectory.__init__(self,trajectory_node)
    self.start_point = start
    self.end_point = end
    self.tg = TrajectoryGenerator()
    
    
    

  def set_start(self, point):
    self.start_point = point

  def set_end(self, point):
    self.end_point = point 

  def begin(self):
    self.__set_done(False)

  def loop(self,start_time):
    self.dist = self.tg.get_distance(self.start_point, self.end_point)
    if self.dist == 0:
      self.t_f = 0
      self.constant = 0
      self.e_t = [0.,0.,0.]
    else:
      n = [0.,0.,0.]
      for i in range(0,3):
        n[i] = self.end_point[i] - self.start_point[i]
      self.e_t = self.tg.get_direction(n) 
      self.t_f = math.sqrt(6*self.dist/0.9*self.a_max)
      self.constant = -2.0/self.t_f**3.0 * self.dist
    r = 10.0
    rate = rospy.Rate(10)
    time = start_time
    while not rospy.is_shutdown() and not self.is_done():
      #get position, velocity and acceleratio at time t
      outpos = self.get_position(time)
      outpos.append(0)
      outvelo = self.get_velocity(time)
      outvelo.append(0)
      outacc = self.get_acceleration(time)
      outacc.append(0)
      #get the corresponding message and publish it
      outmsg = self.tg.get_message(outpos, outvelo, outacc)
      self.trajectory_node.send_msg(outmsg)
      self.trajectory_node.send_permission(False)
      rate.sleep()
      time += 1/r
      #when done, hover at end_point
      if time >= self.t_f:
        self.__set_done(True)
        end = self.end_point
        end.append(0)
        outmsg = self.tg.get_message(self.end_point, [0.0,0.0,0.0,0.0], [0.0,0.0,0.0,0.0])
        self.trajectory_node.send_msg(outmsg)
        self.trajectory_node.send_permission(False)

      
  def __set_done(self,boolean):
    self.done = boolean

  
  #calculates position at time t
  def get_position(self,t):
    outpos = [0.0,0.0,0.0]
    s = self.get_s(t)
    for i in range(0,3):
      outpos[i] = self.start_point[i] + s * self.e_t[i]
    return outpos

  def get_s(self,t):
    return t**2.0 * self.constant * (t-1.5*self.t_f)

  #calculates velocity at time t
  def get_velocity(self,t):
    outvelo = [0.0,0.0,0.0]
    s_d = self.get_s_d(t)
    for i in range(0,3):
      outvelo[i] = s_d * self.e_t[i]
    return outvelo

  def get_s_d (self,t):
    return self.constant * (3 * t**2.0 - 3 * self.t_f * t)
  
  #calculates acceleration at time t
  def get_acceleration(self,t):
    outacc = [0.0,0.0,0.0]
    s_dd = self.get_s_dd(t)
    for i in range(0,3):
      outacc[i] = s_dd * self.e_t[i]
    return outacc

  def get_s_dd (self,t):
    return self.constant * (6 * t - 3 * self.t_f )

  def is_done(self):
    return self.done


if __name__ == '__main__':
  try:
    rospy.sleep(3.)
    traj = TrajectoryNode()
    traj.send_permission(True)
    StraightLineGen(traj,[0.,0.,0.2],[0.,0.,1.]).loop(0.)
    rospy.sleep(10.)
    StraightLineGen(traj,[0.,0.,1.],[0.,0.8,1.]).loop(0.)
  except rospy.ROSInterruptException:
    pass




  
