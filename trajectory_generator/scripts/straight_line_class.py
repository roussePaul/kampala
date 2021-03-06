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
#generates a straight line between startpoint and endpoint with velocity zero at the start and endpoint
#respects constraints on acceleration

class StraightLineGen(Trajectory):
  
  done = False
  a_max = 9.81/3.
 

  def __init__(self,trajectory_node,start,end):
    Trajectory.__init__(self,trajectory_node)
    self.start_point = start
    self.end_point = end
    self.tg = TrajectoryGenerator()
    self.dist = self.tg.get_distance(self.start_point, self.end_point)
    n = [0.,0.,0.]
    for i in range(0,3):
      n[i] = self.end_point[i] - self.start_point[i]
    self.e_t = self.tg.get_direction(n) 
    self.t_f = math.sqrt(6*self.dist/0.9*self.a_max)
    self.constant = -2.0/self.t_f**3.0 * self.dist
    

  def begin(self):
    self.__set_done(False)

  def loop(self,start_time):
    r = 10.0
    rate = rospy.Rate(10)
    time = start_time
    while not rospy.is_shutdown() and not self.is_done():
      outpos = self.get_position(time)
      outpos.append(0)
      outvelo = self.get_velocity(time)
      outvelo.append(0)
      outacc = self.get_acceleration(time)
      outacc.append(0)
      outmsg = self.tg.get_message(outpos, outvelo, outacc)
      self.trajectory_node.send_msg(outmsg)
      self.trajectory_node.send_permission(False)
      rate.sleep()
      time += 1/r
      if time >= self.t_f:
        self.__set_done(True)
        end = self.end_point
        end.append(0)
        outmsg = self.tg.get_message(self.end_point, [0.0,0.0,0.0,0.0], [0.0,0.0,0.0,0.0])
        self.trajectory_node.send_msg(outmsg)
        self.trajectory_node.send_permission(False)

      
  def __set_done(self,boolean):
    self.done = boolean

  
  
  def get_position(self,t):
    outpos = [0.0,0.0,0.0]
    s = self.get_s(t)
    for i in range(0,3):
      outpos[i] = self.start_point[i] + s * self.e_t[i]
    return outpos

  def get_s(self,t):
    return t**2.0 * self.constant * (t-1.5*self.t_f)


  def get_velocity(self,t):
    outvelo = [0.0,0.0,0.0]
    s_d = self.get_s_d(t)
    for i in range(0,3):
      outvelo[i] = s_d * self.e_t[i]
    return outvelo

  def get_s_d (self,t):
    return self.constant * (3 * t**2.0 - 3 * self.t_f * t)

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
    traj = TrajectoryNode()
    traj.send_permission(True)
    rospy.sleep(5.)
    StraightLineGen(traj,[0.,0.,0.],[0.,0.,0.6]).loop(0.)
    rospy.sleep(15.)
    StraightLineGen(traj,[0.,0.,0.6],[0., 4., 0.6]).loop(0.)  

  except rospy.ROSInterruptException:
    pass




  
