#!/usr/bin/env python
#This file contains a class consisting of functions that may be used when generating trajectories
#The functions have been found to be useful earlier.
#The functions include 
#rotating a coordinate system by angles [theta_x, theta_y, theta_z]-
#offseting a position by a certain vector [x, y, z]-
#generating an arc with radius R and angular velocity w in the xy-plane given an angle phi measured relative to the x-axis-
#generating a straight line between the origin and a certain point [x,y,z] with velocities equal to zero at the origin and the final point

import rospy
import sys
import ast
import math
import numpy as np
from numpy import linalg as la
from mocap.msg import QuadPositionDerived



class TrajectoryGenerator():

  def __init__(self):
    pass

#Method that rotates a vector by angles theta = [theta_x, theta_y, theta_z], theta is given in radians
#Rotations are done in the order z,y,x, i.e. [x',y',z'] =R_x R_y R_z [x,y,z] (each rotation is clockwise)
  def rotate_vector(self,vector,theta):
    vector = np.matrix(vector)
    vector = np.transpose(vector)
    vector = self.__R_z(vector, theta[2])
    vector = self.__R_y(vector, theta[1])
    vector = self.__R_x(vector, theta[0])
    return np.transpose(vector)

  def __R_z(self,vector,theta):
    R = np.matrix([[math.cos(theta), -math.sin(theta), 0.0], [math.sin(theta), math.cos(theta), 0.0], [0.0, 0.0, 1.0]])
    return np.dot(R,vector)

  def __R_y(self,vector,theta):
    R = np.matrix([[math.cos(theta), 0.0, math.sin(theta)], [0.0 ,1.0 ,0.0], [-math.sin(theta), 0 ,math.cos(theta)]])
    return np.dot(R,vector)
    
  def __R_x(self,vector,theta):
    R = np.matrix([[1, 0, 0],[ 0, math.cos(theta), -math.sin(theta)],[ 0, math.sin(theta), math.cos(theta)]])
    return np.dot(R,vector)  

#This function offsets a vector by the vector offset
  def offset(self,vector, offset):
    for i in range(0,3):
      vector[i] += offset[i]
    return vector

#This function returns a point on a circle in the xy-plane with radius R and midpoint [0,0,0] given the angle at which it resides
  def get_circle_point(self,R,theta):
    x = R*math.cos(theta)
    y = R*math.sin(theta)
    return [x,y,0]

#This function returns the velocity on a circle in the xy-plane with radius R and midpoint [0,0,0] given the angle at which it resides and the angular velocity
  def get_circle_velocity(self,R,theta,w):
    v_x = -R*w*math.sin(theta)
    v_y = R*w*math.cos(theta)
    return[v_x,v_y,0]
  
#This function returns the acceleration on a circle in the xy-plane with radius R and midpoint [0,0,0] given the angle at which it resides, the angular velocity and the angular acceleration
  def get_circle_acc(self,R,theta,w,alpha):
    a_x = -R*alpha*math.sin(theta) - R*w**2.0*math.cos(theta)
    a_y = R*alpha*math.cos(theta) - R*w**2.0*math.sin(theta)
    return[a_x,a_y,0]


#This function adjusts the yaw so that the quad faces in the direction of the vector given as a parameter (which does not have to be a unit vector)
#Observe that this will return 90 degrees if the vector is parallel to the z-axis
  def adjust_yaw(self, vector):
    e_t = vector/la.norm(vector)
    phi = math.acos(np.dot([1,0,0],e_t))
    return math.degrees(phi)
    
  def get_norm(self,vector):
    return la.norm(vector)

  def get_distance(self,vector1,vector2):
    vector = [0.,0.,0.]
    for i in range(0,3):
      vector[i] = vector1[i] - vector2[i]
    return la.norm(vector)
      

  def get_direction(self,vector):
    return vector/la.norm(vector)

  def get_direction2(self,vector1,vector2):
    e_r = [0.,0.,0.]
    for i in range(0,3):
      e_r[i] = vector1[i]-vector2[i]
    norm = la.norm(e_r)
    if norm == 0. :
      return [0.,0.,0.]
    else:
      return e_r/la.norm(e_r)

  def is_zero_vector(self,vector):
    if la.norm(vector) == 0.:
      return True
    else:
      return False

  def get_message(self,pos,velo,acc):
    msg = QuadPositionDerived()
    msg.x = pos[0]
    msg.y = pos[1]
    msg.z = pos[2]
    msg.yaw = pos[3]
    msg.x_vel = velo[0]
    msg.y_vel = velo[1]
    msg.z_vel = velo[2]
    msg.yaw_vel = velo[3]
    msg.x_acc = velo[0]
    msg.y_acc = velo[1]
    msg.z_acc = velo[2]
    msg.yaw_acc = velo[3]
    return msg

  def get_projection(self,vector,e_t):
    return math.acos(np.dot(vector,e_t)/la.norm(vector))

  def vector_to_list(self,vector):
    vector = vector.tolist()
    lis = vector[0]
    return lis
 


if __name__ == '__main__':
  try:
    tg = TrajectoryGenerator()
    print(tg.get_projection([3,2,3],[1,0,0]))
  except rospy.ROSInterruptException:
    pass
