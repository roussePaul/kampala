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

  ##@param vector: the vector in R^3 to be rotated represented by a list
  ##@param theta: a vector of three rotation angles represented by a list
  ##@return the rotated vector
  def rotate_vector(self,vector,theta):
    """This function rotates a vector by angles theta = [theta_x, theta_y, theta_z], theta is given in radians
    Rotations are done in the order z,y,x, i.e. [x',y',z'] =R_x R_y R_z [x,y,z] (each rotation is clockwise)."""
    vector = np.matrix(vector)
    vector = np.transpose(vector)
    vector = self.__R_z(vector, theta[2])
    vector = self.__R_y(vector, theta[1])
    vector = self.__R_x(vector, theta[0])
    return np.transpose(vector)

  ##@param vector: the vector in R^3 to be rotated represented by a list
  ##@param theta: the angle of rotation about the z-axis
  ##@return the rotated vector  
  def __R_z(self,vector,theta):
    R = np.matrix([[math.cos(theta), -math.sin(theta), 0.0], [math.sin(theta), math.cos(theta), 0.0], [0.0, 0.0, 1.0]])
    return np.dot(R,vector)

  ##@param vector: the vector in R^3 to be rotated represented by a list
  ##@param theta: the angle of rotation about the y-axis
  ##@return the rotated vector  
  def __R_y(self,vector,theta):
    R = np.matrix([[math.cos(theta), 0.0, math.sin(theta)], [0.0 ,1.0 ,0.0], [-math.sin(theta), 0 ,math.cos(theta)]])
    return np.dot(R,vector)
    

  ##@param vector: the vector in R^3 to be rotated represented by a list
  ##@param theta: the angle of rotation about the x-axis
  ##@return the rotated vector  
  def __R_x(self,vector,theta):
    R = np.matrix([[1, 0, 0],[ 0, math.cos(theta), -math.sin(theta)],[ 0, math.sin(theta), math.cos(theta)]])
    return np.dot(R,vector)  


  ##@param vector: the vector in R^3 to be moved represented by a list
  ##@param offset: a vector in R^3 used as an offset represented by a list
  ##@return the vector vector+offset represented by a list  
  def offset(self,vector, offset):
    """This function offsets a vector by the vector offset."""
    for i in range(0,3):
      vector[i] += offset[i]
    return vector

  ##@param R: the radius of the circle
  ##@param theta: the angle that the vector to the point makes with the x-axis
  ##@return the point on the circle
  def get_circle_point(self,R,theta):
    """This function returns a point on a circle in the xy-plane with radius R and midpoint [0,0,0] given the angle at which it resides."""
    x = R*math.cos(theta)
    y = R*math.sin(theta)
    return [x,y,0]

  ##@param R: the radius of the circle
  ##@param theta: the angle that the vector to the point makes with the x-axis
  ##@param w: the angular velocity
  ##@return the velocity corresponding to the circular motion
  def get_circle_velocity(self,R,theta,w):
    """This function returns the velocity on a circle in the xy-plane with radius R and midpoint [0,0,0] given the angle at which it resides and the angular velocity."""
    v_x = -R*w*math.sin(theta)
    v_y = R*w*math.cos(theta)
    return[v_x,v_y,0]
  
  ##@param R: the radius of the circle
  ##@param theta: the angle that the vector to the point makes with the x-axis
  ##@param w: the angular velocity
  ##@param alpha: the angular acceleration
  ##@return the acceleration corresponding to the circular motion
  def get_circle_acc(self,R,theta,w,alpha):
    """This function returns the acceleration on a circle in the xy-plane with radius R and midpoint [0,0,0] given the angle at which it resides, the angular velocity and the angular acceleration."""
    a_x = -R*alpha*math.sin(theta) - R*w**2.0*math.cos(theta)
    a_y = R*alpha*math.cos(theta) - R*w**2.0*math.sin(theta)
    return[a_x,a_y,0]


  ##@param vector: a vector that the yaw of the quad should be aligned with
  ##@return the corresponding yaw
  def adjust_yaw(self, vector):
    """This function adjusts the yaw so that the quad faces in the direction of the vector given as a parameter (which does not have to be a unit vector)
    Observe that this will return 90 degrees if the vector is parallel to the z-axis."""
    e_t = vector/la.norm(vector)
    phi = math.acos(np.dot([1,0,0],e_t))
    return math.degrees(phi)
    
  ##@param vector: a vector in R^3 represented by a list
  ##@return the norm of the vector
  def get_norm(self,vector):
    """This function returns the norm of a vector."""
    return la.norm(vector)

  ##@param vector1: a vector in R^3 represented by a list
  ##@param vector2: a vector in R^3 represented by a list
  ##@return the distance between the two points associated with the vectors
  def get_distance(self,vector1,vector2):
    """This function returns the distance between two points in R^3."""
    vector = [0.,0.,0.]
    for i in range(0,3):
      vector[i] = vector1[i] - vector2[i]
    return la.norm(vector)
      
  ##@param vector: a vector in R^3 represented by a list
  ##@return the direction of the vector
  def get_direction(self,vector):
    """This function returns the unit vector associated with a given vector. If the given 
    vector is the zero vector the zero vector is returned."""
    if la.norm(vector) != 0.:
      return vector/la.norm(vector)
    else:
      return [0.,0.,0.]

  ##@param vec1: a vector in R^3 represented by a list
  ##@param vec2: a vector in R^3 represented by a list
  ##@return the difference vec1 - vec2
  def get_vector(self,vec1,vec2):
    """Returns vector = vec1 - vec2."""
    vector = [0.,0.,0.]
    for i in range(0,3):
      vector[i] = vec1[i] - vec2[i]
    return vector
      
  ##@param vector1: a vector in R^3 represented by a list
  ##@param vector2: a vector in R^3 represented by a list
  ##@return the direction from vector2 towards vector1 
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

  ##@param vector: a vector in R^3 represented by a list
  ##@return true if vector is the zero vector and false in all other cases
  def is_zero_vector(self,vector):
    """This function checks if a given vector is the zero vector."""
    if la.norm(vector) == 0.:
      return True
    else:
      return False


  ##@param pos: the position as a vector in R^4 where the last component is the yaw
  ##@param velo: the velocity as a vector in R^4 where the last component is the yaw rate
  ##@param acc: the acceleration as a vector in R^4 where the last component is the change of the yaw rate
  ##@return the ROS message understood by the controller
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
 
  ##@param vector: a vector in R^3 represented by a list
  ##@param e_t: a unit vector in R^3 represented by a list
  ##@return the projection of vector on e_t
  def get_projection(self,vector,e_t):
    """This function returns the projection of a vector onto a certain unit vector."""
    return np.dot(vector,e_t)

  ##@param vector: a vector represented by a numpy matrix
  ##@return the list corresponding to the matrix
  def vector_to_list(self,vector):
    """This function converts a vector into a list."""
    vector = vector.tolist()
    lis = vector[0]
    return lis
 



