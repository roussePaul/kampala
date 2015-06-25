#!/usr/bin/env python
import rospy
import sys
import ast
import math
import numpy as np
from controller.msg import Permission 
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
    self.radius = rospy.get_param("trajectory_generator/radius",0.8)
    self.velo = rospy.get_param("trajectory_generator/velo",0.4)
    self.theta = rospy.get_param("trajectory_generator/theta",0)

  def get_tilted_circle(self):
    tilted_midpoint = self.inverse_transform(self.midpoint,self.theta) 
    a_max = 0.6**2.0/0.8
    v_max = (self.radius*a_max)**(0.5)
    if self.velo > v_max:
      self.velo = v_max
    start_point = self.go_to_start(tilted_midpoint)
    pub = rospy.Publisher('trajectory_gen/target',QuadPositionDerived, queue_size=10)
    sec_pub = rospy.Publisher('trajectory_gen/done', Permission, queue_size=10)
    rospy.init_node('TG',anonymous=True)
    r = 10.0
    rate = rospy.Rate(r)
    period = 2*math.pi*self.radius/self.velo
    points_in_period = int(period * r)
    time = 0.0
    counter = 0
    rospy.sleep(10)
    while not rospy.is_shutdown() and not self.done:
      out_pos = self.transform_coordinates(self.get_outpos(self.radius, self.velo, tilted_midpoint, time),self.theta)
      #print(out_pos)
      out_velo = self.transform_coordinates(self.get_outvelo(self.radius, self.velo, time),self.theta)
      out_acc = self.transform_coordinates(self.get_outacc(self.radius, self.velo, time),self.theta)
      out_msg = QuadPositionDerived()
      out_msg.found_body = True 
      out_msg.x = out_pos[0]
      out_msg.y = out_pos[1]
      out_msg.z = out_pos[2]
      out_msg.pitch = 0
      out_msg.roll = 0
      out_msg.yaw = self.get_yaw(out_velo)
      out_msg.x_vel = out_velo[0]  
      out_msg.y_vel = out_velo[1]
      out_msg.z_vel = out_velo[2]
      out_msg.pitch_vel = 0
      out_msg.roll_vel = 0
      out_msg.yaw_vel = 0
      out_msg.x_acc = out_acc[0]
      out_msg.y_acc = out_acc[1]
      out_msg.z_acc = out_acc[2]
      out_msg.pitch_acc = 0
      out_msg.roll_acc = 0
      out_msg.yaw_acc = 0
      out_msg.time_diff = 1/r
      pub.publish(out_msg)
      sec_pub.publish(Permission(False))
      time += 1.0/r
      counter += 1
      rate.sleep()
      if counter == 3*points_in_period:
        rospy.set_param("trajectory_generator/start_point",out_pos)
        rospy.set_param("trajectory_generator/end_point",[0.0,0.0,0.6])
        sl_gen = StraightLineGen() 
        sl_gen.generate()
        rospy.sleep(0.1)
        self.turn(pub, [0.0,0.0,0.6],180)
        self.done = True
    sec_pub.publish(Permission(True))
    
    


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

  def transform_coordinates(self,coordinates,theta):             #transforms from tilted frame to SML-frame
    x_temp = coordinates[0] * math.cos(theta) + coordinates[2] * math.sin(theta)
    z_temp = coordinates[2] * math.cos(theta) - coordinates[0] * math.sin(theta)
    return [x_temp,coordinates[1],z_temp]  

  def inverse_transform(self,coordinates,theta):  #transforms from SML-frame to tilted frame
    x_temp = coordinates[0] * math.cos(theta) - coordinates[2] * math.sin(theta)
    z_temp = coordinates[0] * math.sin(theta) + coordinates[2] * math.cos(theta)
    return [x_temp, coordinates[1], z_temp]

  def get_yaw(self,v):
    yaw = math.acos(np.dot([1,0,0],v))
    return math.degrees(yaw)    

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
    

  def turn(self,pub, pos ,yaw):
    out_msg = QuadPositionDerived()
    out_msg.x = pos[0]
    out_msg.y = pos[1]
    out_msg.z = pos[2]
    out_msg.yaw = yaw
    out_msg.x_vel = 0  
    out_msg.y_vel = 0
    out_msg.z_vel = 0
    out_msg.yaw_vel = 0
    out_msg.x_acc = 0
    out_msg.y_acc = 0
    out_msg.z_acc = 0
    out_msg.yaw_acc = 0
    pub.publish(out_msg)
    rospy.sleep(1)
    
    

if __name__ == '__main__':
  #rospy.sleep(10)
  try:
    circle_generator = CircleGen()
    circle_generator.get_tilted_circle()  
  except rospy.ROSInterruptException:
    pass
