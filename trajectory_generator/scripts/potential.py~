#!/usr/bin/env python
import rospy
import sys
import ast
import math
import numpy

class PotGen():

  def __init__(self):
    pass

  def get_acceleration(self, distance, direction):
    alpha = 1.
    r_0  = 1.
    acc = [0.,0.,0.]
    for i in range(0,3):
      acc[i] = alpha/r_0 * math.e**(-distance/r_0) * direction[i] 
    return acc

  def get_velocity(self, current_velocity, acceleration,dt):
    velocity = [0.,0.,0.]
    for i in range(0,3):
      velocity[i] = current_velocity[i] + acceleration[i]*dt
    return velocity
  
  def get_position(self, current_position,velocity, dt):
    position = [0.,0.,0.]
    for i in range(0,3):
      position[i] = current_position[i] + velocity[i] *dt    
    return position


