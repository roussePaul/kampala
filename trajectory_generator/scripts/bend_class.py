#!/usr/bin/env python
import rospy
import sys
import ast
import math
import numpy
from mocap.msg import QuadPositionDerived
from straight_line_class import StraightLineGen

#This script generates the points, velocities and accelerations to be used as a reference for the 
#controller to to get the quad to move in a circle.
#Given a midpoint, radius and speed a circle is generated such that the quad moves at a constant speed.
#Constraints on maximum velocity and acceleration are used.
#Everyhthing is calculated in a coordinatesystem that is rotated by an angle of theta about the z-axis of the SML-frame. The method transform_coordinates transforms a vector given in the rotated frame into the corresponding vector in the SML-frame. 

class CircleGen:
  
  def __init__(self):
    self.done = False
    self.midpoint = rospy.get_param("trajectory_generator/midpoint",[0.0,0.0,0.6])
    if type(self.midpoint) is str:
      self.midpoint = ast.literal_eval(self.midpoint)
    self.start = rospy.get_param("trajectory_generator/start",[0.4,0.0,0.6])
    if type(self.start) is str:
     self.start = ast.literal_eval(self.start)
    self.radius = self.get_norm([self.midpoint[0]-self.start[0],self.midpoint[1]-self.start[1],self.midpoint[2]-self.start[2]])
    self.initial_velo = rospy.get_param("trajectory_generator/initial_velo",[0.0,0.2,0.2])
    if type(self.initial_velo) is str:
      self.initial_velo = ast.literal_eval(self.initial_velo)
    self.velo = self.get_norm(self.initial_velo)
    #define new coordinates
    self.xp = self.get_dir([self.start[0]-self.midpoint[0],self.start[1]-self.midpoint[1],self.start[2]-self.midpoint[2]])
    self.yp = self.get_dir(self.initial_velo)
    self.zp = numpy.cross(self.xp,self.yp)
    self.new_frame = [self.xp,self.yp,self.zp]
    self.psi = rospy.get_param("trajectory_generator/psi",2*math.pi) #angle of rotation about initial e_n direction

#Observe that initial_velo always has to be perpendicular to the vector pointing from start to #midpoint!!!

  def get_tilted_circle(self):
    tilted_midpoint = self.inverse_transform(self.midpoint) 
    a_max = 0.6**2.0/0.8
    v_max = (self.radius*a_max)**(0.5)
    if self.velo > v_max:
      self.velo = v_max
    pub = rospy.Publisher('trajectory_gen/target',QuadPositionDerived, queue_size=10)
    rospy.init_node('TG')
    r = 25
    rate = rospy.Rate(r)
    period = self.psi*self.radius/self.velo
    points_in_period = int(period * r)
    time = 0.0
    counter = 0
    while not rospy.is_shutdown() and not self.done:
      out_pos = self.transform_coordinates(self.get_outpos(self.radius, self.velo, tilted_midpoint, time)) 
      out_velo = self.transform_coordinates(self.get_outvelo(self.radius, self.velo, time))
      out_acc = self.transform_coordinates(self.get_outacc(self.radius, self.velo, time))
      out_msg = QuadPositionDerived()
      out_msg.x = out_pos[0]
      out_msg.y = out_pos[1]
      out_msg.z = out_pos[2]
      out_msg.yaw = self.get_yaw(self.velo,time,self.radius)
      out_msg.x_vel = out_velo[0]  
      out_msg.y_vel = out_velo[1]
      out_msg.z_vel = out_velo[2]
      out_msg.yaw_vel = 0
      out_msg.x_acc = out_acc[0]
      out_msg.y_acc = out_acc[1]
      out_msg.z_acc = out_acc[2]
      out_msg.yaw_acc = 0
      pub.publish(out_msg)
      time += 1.0/r
      counter += 1
      rate.sleep()
      if counter == points_in_period and time == period:
        self.done = True
      elif counter == points_in_period:
        out_pos = self.transform_coordinates(self.get_outpos(self.radius, self.velo, tilted_midpoint, period))
        out_velo = self.transform_coordinates(self.get_outvelo(self.radius, self.velo, period))
        out_acc = self.transform_coordinates(self.get_outacc(self.radius, self.velo, period))
        out_msg = QuadPositionDerived()
        out_msg.x = out_pos[0]
        out_msg.y = out_pos[1]
        out_msg.z = out_pos[2]
        out_msg.yaw = self.get_yaw(self.velo,time,self.radius)
        out_msg.x_vel = out_velo[0]  
        out_msg.y_vel = out_velo[1]
        out_msg.z_vel = out_velo[2]
        out_msg.yaw_vel = 0
        out_msg.x_acc = out_acc[0]
        out_msg.y_acc = out_acc[1]
        out_msg.z_acc = out_acc[2]
        out_msg.yaw_acc = 0
        pub.publish(out_msg)
        self.done = True
        rate.sleep()

  def get_outpos(self, R,v,r_m,t):
    outpos = [0.0,0.0,0.0]
    outpos[0] = r_m[0] + R*math.cos(v*t/R)
    outpos[1] = r_m[1] + R*math.sin(v*t/R)
    outpos[2] = r_m[2]
    return outpos

  def get_outvelo(self,R,v,t):
    outvelo = [0.0,0.0,0.0]
    outvelo[0] = -v*math.sin(v*t/R)
    outvelo[1] = v*math.cos(v*t/R)
    outvelo[2] = 0
    return outvelo

  def get_outacc(self,R,v,t):
    outacc = [0.0,0.0,0.0]
    outacc[0] = -v**2.0/R*math.cos(v*t/R)
    outacc[1] = -v**2.0/R*math.sin(v*t/R)
    outacc[2] = 0
    return outacc

  def transform_coordinates(self,coordinates):   #transforms from tilted frame to SML-frame
    transformed_coords = [0.0,0.0,0.0]
    for j in range(0,3):
      for i in range(0,3):
        transformed_coords[j] += coordinates[i] * self.new_frame[i][j]
    return transformed_coords

  def inverse_transform(self,coordinates):  #transforms from SML-frame to tilted frame
    new_x = numpy.dot(coordinates,self.xp)
    new_y = numpy.dot(coordinates,self.yp) 
    new_z = numpy.dot(coordinates,self.zp)
    return [new_x,new_y,new_z]

  def get_yaw(self,v,t,R):
    yaw = math.pi/2 + v*t/R
    is_positive = yaw > 0 
    if is_positive:
      while yaw > math.pi:
        yaw -= 2*math.pi
    else:
      while yaw < -(math.pi):
        yaw += 2*math.pi
    return yaw    

  def go_to_start(self,midpoint):
    target_point = [0.0,0.0,0.0]
    target_point[0] = midpoint[0] + self.radius
    target_point[1] = midpoint[1]
    target_point[2] = midpoint[2]
    outpos = self.transform_coordinates(target_point,self.theta)
    rospy.set_param("trajectory_generator/start_point",[0.0,0.0,0.2])
    rospy.set_param("trajectory_generator/end_point",outpos)
    sl_gen = StraightLineGen()
    sl_gen.generate()
    return outpos
    
  def get_norm(self,a):
    return math.sqrt(a[0]**2.0+a[1]**2.0+a[2]**2.0)    

  def get_dir(self,a): 
    norm = self.get_norm(a)
    e = [0.0,0.0,0.0]
    for i in range(0,3):
      e[i] = (a[i]/norm)
    return e

if __name__ == '__main__':
  try:
    circle_generator = CircleGen()
    circle_generator.get_tilted_circle()  
  except rospy.ROSInterruptException:
    pass
