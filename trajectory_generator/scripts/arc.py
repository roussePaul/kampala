#!/usr/bin/env python

#!/usr/bin/env python
import rospy
import sys
import ast
import math
import numpy
from mocap.msg import QuadPositionDerived
from trajectory_generato import TrajectoryGenerator

#This script generates the points, velocities and accelerations to be used as a reference for the 
#controller to to get the quad to move in a circle.
#Given a midpoint, radius and speed a circle is generated such that the quad moves at a constant speed.
#Constraints on maximum velocity and acceleration are used.
#Everyhthing is calculated in a coordinatesystem that is rotated by an angle of theta about the z-axis of the SML-frame. The method transform_coordinates transforms a vector given in the rotated frame into the corresponding vector in the SML-frame. 

class ArcGenerator:
  
  def __init__(self):
    self.tg = TrajectoryGenerator()
    self.midpoint = rospy.get_param("trajectory_generator/midpoint",[0.0,0.0,0.6])
    if type(self.midpoint) is str:
      self.midpoint = ast.literal_eval(self.midpoint)
    self.start = rospy.get_param("trajectory_generator/start",[0.4,0.0,0.6])
    if type(self.start) is str:
     self.start = ast.literal_eval(self.start)
    n = [self.start[0]-self.midpoint[0],self.start[1]-self.midpoint[1],self.start[0]-self.midpoint[2]]
    self.radius = self.tg.get_norm(n)
    self.initial_velo = rospy.get_param("trajectory_generator/initial_velo",[0.0,0.2,0.2])
    if type(self.initial_velo) is str:
      self.initial_velo = ast.literal_eval(self.initial_velo)
    self.velo = self.tg.get_norm(self.initial_velo)
    #define new coordinates
    self.e_n = self.tg.get_direction(n)
    self.yp = self.tg.get_direction(self.initial_velo)
    self.zp = numpy.cross(self.e_n,self.yp)
    self.psi = rospy.get_param("trajectory_generator/psi",4*math.pi) #angle of rotation about initial e_n direction

  def generate(self):
    a_max = 0.6**2.0/0.8
    v_max = (self.radius*a_max)**(0.5)
    if self.velo > v_max:
      self.velo = v_max
    w = self.radius * self.velo
    pub = rospy.Publisher('trajectory_gen/target',QuadPositionDerived, queue_size=10)
    rospy.init_node('TG')
    theta_z = self.tg.get_projection([0,0,1],self.e_n)
    r = 10.0
    rate = rospy.Rate(r)
    time = 0.0
    while not rospy.is_shutdown() and not w*time > self.psi:
      outpos = self.tg.get_circle_point(self.radius,w*time)
      outpos = self.tg.rotate_vector(outpos,[0,0,theta_z])
      outpos = self.tg.offset(outpos,self.midpoint)
      outpos.append(self.tg.adjust_yaw([1,0,0]))
      outvelo = self.tg.get_circle_velocity(self.radius,w*time,w)
      outvelo = self.tg.rotate_vector(outvelo,[0,0,theta_z])
      outvelo.append(0)
      outacc = self.tg.get_circle_acc(self.radius,w*time,w,0)
      outacc = self.tg.rotate_vector(outacc,[0,0,theta_z])
      outacc.append(0)
      outmsg = self.tg.get_message(outpos,outvelo,outacc)
      pub.publish(outmsg)
      rate.sleep()
      time += 1/r
    print("done")
if __name__  == '__main__':
  ArcGenerator().generate()
    
