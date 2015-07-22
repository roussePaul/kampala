#!/usr/bin/env python

import rospy
import sys
import ast
import math
import numpy as np
from numpy import linalg as la
from mocap.msg import QuadPositionDerived



class TrajectoryGenerator():
  """This file contains a class consisting of functions that may be used when generating trajectories
  The functions have been found to be useful earlier.
  The functions include 
  -rotating a coordinate system by angles [theta_x, theta_y, theta_z]
  -offseting a position by a certain vector [x, y, z]
  -generating an arc with radius R and angular velocity w in the xy-plane given an angle phi measured relative to the x-axis.
  There are also certain vector operations that occur often when generating trajectories. Also, a method that given 
  position, velocity and acceleration, generates the correct rosmsg exists."""



  def __init__(self):
    pass


  def rotate_vector(self,vector,theta):
    """This function rotates a vector by angles theta = [theta_x, theta_y, theta_z], theta is given in radians
    Rotations are done in the order z,y,x, i.e. [x',y',z'] =R_x R_y R_z [x,y,z] (each rotation is clockwise)."""
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


  def offset(self,vector, offset):
    """This function offsets a vector by the vector offset."""
    for i in range(0,3):
      vector[i] += offset[i]
    return vector

  def get_circle_point(self,R,theta):
    """This function returns a point on a circle in the xy-plane with radius R and midpoint [0,0,0] given the angle at which it resides."""
    x = R*math.cos(theta)
    y = R*math.sin(theta)
    return [x,y,0]


  def get_circle_velocity(self,R,theta,w):
    """This function returns the velocity on a circle in the xy-plane with radius R and midpoint [0,0,0] given the angle at which it resides and the angular velocity."""
    v_x = -R*w*math.sin(theta)
    v_y = R*w*math.cos(theta)
    return[v_x,v_y,0]
  

  def get_circle_acc(self,R,theta,w,alpha):
    """This function returns the acceleration on a circle in the xy-plane with radius R and midpoint [0,0,0] given the angle at which it resides, the angular velocity and the angular acceleration."""
    a_x = -R*alpha*math.sin(theta) - R*w**2.0*math.cos(theta)
    a_y = R*alpha*math.cos(theta) - R*w**2.0*math.sin(theta)
    return[a_x,a_y,0]



  def adjust_yaw(self, vector):
    """This function adjusts the yaw so that the quad faces in the direction of the vector given as a parameter (which does not have to be a unit vector)
    Observe that this will return 90 degrees if the vector is parallel to the z-axis."""
    e_t = vector/la.norm(vector)
    phi = math.acos(np.dot([1,0,0],e_t))
    return math.degrees(phi)
    
  def get_norm(self,vector):
    """This function returns the norm of a vector."""
    return la.norm(vector)

  def get_distance(self,vector1,vector2):
    """This function returns the distance between two points in R^3."""
    vector = [0.,0.,0.]
    for i in range(0,3):
      vector[i] = vector1[i] - vector2[i]
    return la.norm(vector)
      
  def get_direction(self,vector):
    """This function returns the unit vector associated with a given vector."""
    return vector/la.norm(vector)

  def get_vector(self,vec1,vec2):
    """Returns vector = vec1 - vec2."""
    vector = [0.,0.,0.]
    for i in range(0,3):
      vector[i] = vec1[i] - vec2[i]
    return vector
      

  def get_direction2(self,vector1,vector2):
    """This function returns the unit vector associated with the vector between two points.
    The direction is away from vector2 towards vector1."""
    e_r = [0.,0.,0.]
    for i in range(0,3):
      e_r[i] = vector1[i]-vector2[i]
    norm = la.norm(e_r)
    if norm == 0. :
      return [0.,0.,0.]
    else:
      return e_r/la.norm(e_r)

  def is_zero_vector(self,vector):
    """This function checks if a given vector is the zero vector."""
    if la.norm(vector) == 0.:
      return True
    else:
      return False

  def get_message(self,pos,velo,acc):
    """This function takes in a position, velocity and acceleration and returns the corresponding #QuadPositionDerived-message for ROS."""
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
    msg.y_acc = acc[1]
    msg.z_acc = acc[2]
    msg.yaw_acc = acc[3]
    return msg

  def get_projection(self,vector,e_t):
    """This function returns the projection of a vector onto a certain unit vector."""
    return np.dot(vector,e_t)

  def vector_to_list(self,vector):
    """This function converts a vector into a list."""
    vector = vector.tolist()
    lis = vector[0]
    return lis
 



