#!/usr/bin/env python



import rospy
import sys
import ast
import math
from trajectory import Trajectory
from controller.msg import Permission
from mocap.msg import QuadPositionDerived
from trajectory_generato import TrajectoryGenerator
from trajectory_node import TrajectoryNode

class StraightLineGen(Trajectory):

  """Generates a straight line between the current position of the drone
  and a point specified by a user. This is used with the gui. It uses the StraightLineGen.
  For documentation see the documentation of StraightLineGen."""

  
  done = False
  a_max = 9.81/3.
 

  def __init__(self,trajectory_node,start,end):
    Trajectory.__init__(self,trajectory_node)
    self.__start_point = start
    self.__end_point = end
    self.tg = TrajectoryGenerator()
    self.__dist = self.tg.get_distance(self.__start_point, self.__end_point)
    n = [0.,0.,0.]
    for i in range(0,3):
      n[i] = self.__end_point[i] - self.__start_point[i]
    self.__e_t = self.tg.get_direction(n) 
    self.__t_f = math.sqrt(6*self.__dist/0.9*self.a_max)
    self.__constant = -2.0/self.__t_f**3.0 * self.__dist
    

  def begin(self):
    self.__set_done(False)

  def loop(self,start_time):
    r = 10.0
    rate = rospy.Rate(10)
    time = start_time
    while not rospy.is_shutdown() and not self.is_done():
      outpos = self.__get_position(time)
      outpos.append(0)
      outvelo = self.__get_velocity(time)
      outvelo.append(0)
      outacc = self.__get_acceleration(time)
      outacc.append(0)
      outmsg = self.tg.get_message(outpos, outvelo, outacc)
      self.trajectory_node.send_msg(outmsg)
      self.trajectory_node.send_permission(False)
      rate.sleep()
      time += 1/r
      if time >= self.__t_f:
        self.__set_done(True)
        end = self.__end_point
        end.append(0)
        outmsg = self.tg.get_message(self.__end_point, [0.0,0.0,0.0,0.0], [0.0,0.0,0.0,0.0])
        self.trajectory_node.send_msg(outmsg)
        self.trajectory_node.send_permission(False)

      
  def __set_done(self,boolean):
    self.done = boolean

  
  
  def __get_position(self,t):
    outpos = [0.0,0.0,0.0]
    s = self.__get_s(t)
    for i in range(0,3):
      outpos[i] = self.__start_point[i] + s * self.__e_t[i]
    return outpos

  def __get_s(self,t):
    return t**2.0 * self.__constant * (t-1.5*self.__t_f)


  def __get_velocity(self,t):
    outvelo = [0.0,0.0,0.0]
    s_d = self.__get_s_d(t)
    for i in range(0,3):
      outvelo[i] = s_d * self.__e_t[i]
    return outvelo

  def __get_s_d (self,t):
    return self.__constant * (3 * t**2.0 - 3 * self.__t_f * t)

  def __get_acceleration(self,t):
    outacc = [0.0,0.0,0.0]
    s_dd = self.__get_s_dd(t)
    for i in range(0,3):
      outacc[i] = s_dd * self.__e_t[i]
    return outacc

  def __get_s_dd (self,t):
    return self.__constant * (6 * t - 3 * self.__t_f )

  def is_done(self):
    return self.done


if __name__ == '__main__':
  try:
    xstart = float(sys.argv[1])
    ystart = float(sys.argv[2])
    zstart = float(sys.argv[3])
    xdest = float(sys.argv[4])
    ydest = float(sys.argv[5])
    zdest = float(sys.argv[6])
    traj = TrajectoryNode()
    traj.send_permission(True)
    StraightLineGen(traj,[xstart,ystart,zstart],[xdest,ydest,zdest]).loop(0.)
  except rospy.ROSInterruptException:
    pass





  
