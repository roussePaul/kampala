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
from circle_acc import AccGen

class ArcGen(Trajectory):
  """This class generates the points, velocities and accelerations to be used as a reference for the 
  controller to to get the quad to move in a circle.
  Given a midpoint, radius (R), starting point on the circle, initial velocity and angle (psi) in 
  radians, an arc of length R*psi is generated with the given midpoint, starting at the specified starting point.
  A plane is defined with its x-axis along the vector pointing from the midpoint to the starting point and its 
  y-axis along the direction of the initial velocity. In this plane the arc is generated in such a way that the
  angle increases from zero to psi and the initial motion is in the local y-direction. 
  Care has to be taken by the user as the input velocity has to be perpendicular 
  to the vector pointing from the starting point to the midpoint. 
  Constraints on maximum velocity and acceleration are used."""

  ##@param done: tells whether or not the trajectory is done  
  done = False
  ##@param a_max: the maximum possible acceleration
  a_max = 0.6**2.0/0.8
  

  def __init__(self,trajectory_node,mid,start,velo,psi):
    """In the constructor all necessary variables are initialized."""
    Trajectory.__init__(self,trajectory_node)
    self.tg = TrajectoryGenerator()
    self.__midpoint = mid
    self.__start = start
    n = [self.__start[0]-self.__midpoint[0],self.__start[1]-self.__midpoint[1],self.__start[2]-self.__midpoint[2]]
    self.__radius = self.tg.get_distance(self.__start,self.__midpoint)
    self.__velo = self.tg.get_norm(velo)
    #define new coordinates using the vector from the midpoint to the startpoint and the velocity vector
    xp = self.tg.get_direction(n)
    yp = self.tg.get_direction(velo)
    self.__tensor = [xp,yp]
    self.__psi = psi 
    self.__w = self.__radius*self.__velo
  

  def begin(self):
    v_max = (self.__radius*self.a_max)**(0.5)
    if self.__velo > v_max:
      self.__velo = v_max
    self.__set_done(False)
    
  ##@param start_time: sets the initial time used for generating the circle
  def loop(self, start_time):
    """This method is called to perform the whole trajectory."""
    time = start_time
    r = 15.0
    rate = rospy.Rate(r)
    while not rospy.is_shutdown() and not self.is_done():
      #get point, velocity and acceleration
      outpos = self.tg.get_circle_point(self.__radius,self.__w*time)
      outpos = self.__transform(outpos)
      outpos = self.tg.offset(outpos,self.__midpoint)
      outpos.append(self.tg.adjust_yaw([1,0,0]))
      outvelo = self.tg.get_circle_velocity(self.__radius,self.__w*time,self.__w)
      outvelo = self.__transform(outvelo)
      outvelo.append(0)
      outacc = self.tg.get_circle_acc(self.__radius,self.__w*time,self.__w,0)
      outacc = self.__transform(outacc)
      outacc.append(0)
      #get the corresponding message and publish it
      outmsg = self.tg.get_message(outpos,outvelo,outacc)
      self.trajectory_node.send_msg(outmsg)
      self.trajectory_node.send_permission(False)
      rate.sleep()
      time += 1/r
      if self.__w*time >= self.__psi:
        #If we are done get the last point and publish it several times before finishing
        outpos = self.tg.get_circle_point(self.__radius,self.__psi)
        outpos = self.__transform(outpos)
        outpos = self.tg.offset(outpos,self.__midpoint)
        outpos.append(self.tg.adjust_yaw([1,0,0]))
        outvelo = self.tg.get_circle_velocity(self.__radius,self.__psi,self.__w)
        outvelo = self.__transform(outvelo)
        outvelo.append(0)
        outacc = self.tg.get_circle_acc(self.__radius,self.__psi,self.__w,0)
        outacc = self.__transform(outacc)
        outacc.append(0)
        outmsg = self.tg.get_message(outpos,outvelo,outacc)
        for i in range(0,5):
          self.trajectory_node.send_msg(outmsg)
          self.trajectory_node.send_permission(False)
          rate.sleep()
          time += 1/r
        self.__set_done(True)

    
  def is_done(self):
    return self.done

  ##@param boolean: set done to boolean
  def __set_done(self,boolean):
    self.done = boolean

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
   
if __name__ == '__main__':
  try:
    rospy.sleep(4.)
    tn = TrajectoryNode()
    sl_gen_1 = StraightLineGen(tn,[0.,0.,0.2],[0.,0.,0.6])
    sl_gen_1.loop(0.)
    rospy.sleep(5.)
    sl_gen_2 = StraightLineGen(tn,[0.,0.,0.6],[0.8,0.,0.6])
    sl_gen_2.loop(0.)
    rospy.sleep(5.)
    acc_gen = AccGen(tn,[0.,0.,0.6],[0.8,0.,0.6],[0.,0.2,0.])
    acc_gen.loop(0.)
    t = acc_gen.get_t_f()
    a_gen = ArcGen(tn,[0.,0.,0.6],[0.8,0.,0.6],[0.,0.2,0.],2*math.pi)
    a_gen.loop(t/2.0)
    
  except rospy.ROSInterruptException:
    pass
  
