#!/usr/bin/env python
import rospy
import sys
import ast
import math
from mocap.msg import QuadPositionDerived

#This script generates the points, velocities and accelerations to be used as a reference for the 
#controller to to get the quad to move in a circle.
#Given a midpoint, radius and speed a circle is generated such that the quad moves at a constant speed.
#Constraints on maximum velocity and acceleration are used.
#Everyhthing is calculated in a coordinatesystem that is rotated by an angle of theta about the z-axis of the SML-frame. The method transform_coordinates transforms a vector given in the rotated frame into the corresponding vector in the SML-frame. 


def get_tilted_circle():
  midpoint = rospy.get_param("trajectory_generator/midpoint",[0.0,0.0,0.6])
  if type(midpoint) is str:
    midpoint = ast.literal_eval(midpoint)
  radius = rospy.get_param("trajectory_generator/radius",0.8)
  velo = rospy.get_param("trajectory_generator/velo",0.4)
  theta = rospy.get_param("trajectory_generator/theta",math.radians(20))
  tilted_midpoint = inverse_transform(midpoint,theta) 
  v_max = 12.0
  a_max = 9.81/3.0
  v_max_from_a_max = (radius*a_max)**(0.5)
  test = v_max_from_a_max < v_max
  if test and  velo > v_max_from_a_max:
    velo = v_max_from_a_max
  elif not test and velo > v_max:
    velo = v_max
  pub = rospy.Publisher('trajectory_gen/target',QuadPositionDerived, queue_size=10)
  rospy.init_node('circle_gen_tilt',anonymous=True)
  r = 25
  rate = rospy.Rate(r)
  time = 0.0
  #gets the quad to the starting-point
  for j in range(0,15*r):
    out_msg = QuadPositionDerived()
    out_msg.x = 0.0
    out_msg.y = 0.0
    out_msg.z = 0.6
    out_msg.yaw = 0
    out_msg.x_vel = 0  
    out_msg.y_vel = 0
    out_msg.z_vel = 0
    out_msg.yaw_vel = 0
    out_msg.x_acc = 0 
    out_msg.y_acc = 0
    out_msg.z_acc = 0
    out_msg.yaw_acc = 0
    pub.publish(out_msg)
    rate.sleep()
  for i in range(0,5*r):
      out_msg = QuadPositionDerived()
      tilted_midpoint = inverse_transform(midpoint, theta)
      target_point = [0.0,0.0,0.0]
      target_point[0] = tilted_midpoint[0] + radius
      target_point[1] = tilted_midpoint[1]
      target_point[2] = tilted_midpoint[2]
      outpos = transform_coordinates(target_point,theta)
      out_msg.x = outpos[0]
      out_msg.y = outpos[1]
      out_msg.z = outpos[2]
      out_msg.yaw = math.pi/2
      out_msg.x_vel = 0  
      out_msg.y_vel = 0
      out_msg.z_vel = 0
      out_msg.yaw_vel = 0
      out_msg.x_acc = 0 
      out_msg.y_acc = 0
      out_msg.z_acc = 0
      out_msg.yaw_acc = 0
      pub.publish(out_msg)
      rate.sleep()
  while not rospy.is_shutdown():
    #publish "starting point", might be nice to use ptp_gen or some kind of mo_cap feedback
    out_pos = transform_coordinates(get_outpos(radius, velo, tilted_midpoint, time),theta)
    out_velo = transform_coordinates(get_outvelo(radius, velo, time),theta)
    out_acc = transform_coordinates(get_outacc(radius, velo, time),theta)
    out_msg = QuadPositionDerived()
    out_msg.x = out_pos[0]
    out_msg.y = out_pos[1]
    out_msg.z = out_pos[2]
    out_msg.yaw = get_yaw(velo,time,radius)
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
    rate.sleep() 
    
def get_outpos(R,v,r_m,t):
  outpos = [0.0,0.0,0.0]
  outpos[0] = r_m[0] + R*math.cos(v*t/R)
  outpos[1] = r_m[1] + R*math.sin(v*t/R)
  outpos[2] = r_m[2]
  return outpos

def get_outvelo(R,v,t):
  outvelo = [0.0,0.0,0.0]
  outvelo[0] = -v*math.sin(v*t/R)
  outvelo[1] = v*math.cos(v*t/R)
  outvelo[2] = 0
  return outvelo

def get_outacc(R,v,t):
  outacc = [0.0,0.0,0.0]
  outacc[0] = -v**2.0/R*math.cos(v*t/R)
  outacc[1] = -v**2.0/R*math.sin(v*t/R)
  outacc[2] = 0
  return outacc

def transform_coordinates(coordinates,theta):             #transforms from tilted frame to SML-frame
  x_temp = coordinates[0] * math.cos(theta) + coordinates[2] * math.sin(theta)
  z_temp = coordinates[2] * math.cos(theta) - coordinates[0] * math.sin(theta)
  return [x_temp,coordinates[1],z_temp]  

def inverse_transform(coordinates,theta):  #transforms from SML-frame to tilted frame
  x_temp = coordinates[0] * math.cos(theta) - coordinates[2] * math.sin(theta)
  z_temp = coordinates[0] * math.sin(theta) + coordinates[2] * math.cos(theta)
  return [x_temp, coordinates[1], z_temp]

def get_yaw(v,t,R):
  yaw = math.pi/2 - v*t/R
  is_positive = yaw > 0 
  if is_positive:
    while yaw > math.pi:
      yaw -= 2*math.pi
  else:
    while yaw < -(math.pi):
      yaw += 2*math.pi
  return yaw    

if __name__ == '__main__':
  try:
    get_tilted_circle()
  except rospy.ROSInterruptException:
    pass
