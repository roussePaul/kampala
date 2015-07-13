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
from circle_acc import AccGen

#This script generates the points, velocities and accelerations to be used as a reference for the 
#controller to to get the quad to move in a circle.
#Given a midpoint, radius(R), starting point on the circle, initial velocity and angle(psi) in #radians, an arc of length R*psi is generated with the given midpoint, starting at the specified #starting point. Care has to be taken by the user as the input velocity has to be perpendicular #to the vector pointing from the starting point to the midpoint. 
#Constraints on maximum velocity and acceleration are used.

class ArcGen(Trajectory):
  
  done = False
  a_max = 0.6**2.0/0.8
  
  def __init__(self,trajectory_node,mid,start,velo,psi):
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
    #angle of rotation about initial e_n direction
    self.psi = psi 
    self.w = self.radius*self.velo
    self.theta_z = self.tg.get_projection([0,0,1],self.e_n)
  
  def begin(self):
    v_max = (self.radius*self.a_max)**(0.5)
    if self.velo > v_max:
      self.velo = v_max
    self.__set_done(False)
    

  def loop(self, start_time):
    time = start_time
    r = 30.0
    rate = rospy.Rate(r)
    while not rospy.is_shutdown() and not self.is_done():
      #get point, velocity and acceleration
      outpos = self.tg.get_circle_point(self.radius,self.w*time)
      #outpos = self.tg.rotate_vector(outpos,[0,0,self.theta_z])
      #outpos = self.tg.vector_to_list(outpos)
      outpos = self.tg.offset(outpos,self.midpoint)
      outpos.append(self.tg.adjust_yaw([1,0,0]))
      outvelo = self.tg.get_circle_velocity(self.radius,self.w*time,self.w)
      #outvelo = self.tg.rotate_vector(outvelo,[0,0,self.theta_z])
      #outvelo = self.tg.vector_to_list(outvelo)
      outvelo.append(0)
      outacc = self.tg.get_circle_acc(self.radius,self.w*time,self.w,0)
      #outacc = self.tg.rotate_vector(outacc,[0,0,self.theta_z])
      #outacc = self.tg.vector_to_list(outacc)
      outacc.append(0)
      #get the corresponding message and publish it
      outmsg = self.tg.get_message(outpos,outvelo,outacc)
      self.trajectory_node.send_msg(outmsg)
      self.trajectory_node.send_permission(False)
      rate.sleep()
      time += 1/r
      if self.w*time >= self.psi:
        #If we are done get the last point and publish it
        outpos = self.tg.get_circle_point(self.radius,self.psi)
        #outpos = self.tg.rotate_vector(outpos,[0,0,self.theta_z])
        #outpos = self.tg.vector_to_list(outpos)
        outpos = self.tg.offset(outpos,self.midpoint)
        outpos.append(self.tg.adjust_yaw([1,0,0]))
        outvelo = self.tg.get_circle_velocity(self.radius,self.psi,self.w)
        #outvelo = self.tg.rotate_vector(outvelo,[0,0,self.theta_z])
        #outvelo = self.tg.vector_to_list(outvelo)
        outvelo.append(0)
        outacc = self.tg.get_circle_acc(self.radius,self.psi,self.w,0)
        #outacc = self.tg.rotate_vector(outacc,[0,0,self.theta_z])
        #outacc = self.tg.vector_to_list(outacc)
        outacc.append(0)
        outmsg = self.tg.get_message(outpos,outvelo,outacc)
        self.trajectory_node.send_msg(outmsg)
        self.trajectory_node.send_permission(False)
        rate.sleep()
        time += 1/r
        self.__set_done(True)

    
  def is_done(self):
    return self.done

  def __set_done(self,boolean):
    self.done = boolean
   
if __name__ == '__main__':
  try:
    rospy.sleep(4.)
    tn = TrajectoryNode()
    sl_gen_1 = StraightLineGen(tn,[0.,0.,0.2],[0.,0.,0.6])
    sl_gen_1.loop(0.)
    rospy.sleep(2.)
    sl_gen_2 = StraightLineGen(tn,[0.,0.,0.6],[0.8,0.,0.6])
    sl_gen_2.loop(0.)
    rospy.sleep(10.)
    acc_gen = AccGen(tn,[0.,0.,0.6],[0.8,0.,0.6],[0.,0.2,0.])
    t = acc_gen.get_t_f()
    a_gen = ArcGen(tn,[0.,0.,0.6],[0.8,0.,0.6],[0.,0.2,0.],4*math.pi)
    a_gen.loop(t/2.0)
    
  except rospy.ROSInterruptException:
    pass
  
