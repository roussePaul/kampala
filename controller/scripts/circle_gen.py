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
#As of now, yaw is not adjusted and the circle has to lie in a plane defined by z = constant.

def get_circle():
  midpoint = rospy.get_param("trajectory_generator/midpoint",[0.0,0.0,0.6])
  radius = rospy.get_param("trajectory_generator/radius",0.8)
  velo = rospy.get_param("trajectory_generator/velo",0.4)
#  if type(midpoint is str):
 #   midpoint = ast.literal_eval(midpoint)
  v_max = 10.0
  a_max = 10.0
  v_max_from_a_max = (radius*a_max)**(0.5)
  test = v_max_from_a_max < v_max
  if test and  velo > v_max_from_a_max:
    velo = v_max_from_a_max
  elif not test and velo > v_max:
    velo = v_max
  pub = rospy.Publisher('trajectory_gen/target',QuadPositionDerived, queue_size=10)
  rospy.init_node('circle_gen',anonymous=True)
  r = 15
  rate = rospy.Rate(r)
  time = 0.0
  #gets the quad to the starting-point
  for j in range(0,10*r):
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
    got_to_start = True
    rate.sleep()
  for i in range(0,5*r):
      out_msg = QuadPositionDerived()
      out_msg.x = midpoint[0] + radius
      out_msg.y = midpoint[1]
      out_msg.z = midpoint[2]
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
      got_to_start = True
      rate.sleep()
  while not rospy.is_shutdown():
    out_pos = get_outpos(radius, velo, midpoint, time)
    out_velo = get_outvelo(radius, velo, time)
    out_acc = get_outacc(radius, velo, time)
    out_msg = QuadPositionDerived()
    out_msg.x = out_pos[0]
    out_msg.y = out_pos[1]
    out_msg.z = out_pos[2]
#    out_msg.yaw = get_yaw(velo,time,radius)
    out_msg.yaw=0
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

def get_yaw(v,t,R):
  yaw = math.pi/2 + v*t/R
  is_positive = yaw > 0 
  if is_positive:
    while yaw > math.pi:
      yaw -= 2*math.pi
  else:
    while yaw < -(math.pi):
      yaw += 2*math.pi
  return math.degrees(yaw)

if __name__ == '__main__':
  try:
    get_circle()
  except rospy.ROSInterruptException:
    pass
