#!/usr/bin/env python
import rospy
import sys
import ast
import math
import numpy as np
import trajectory_generator
from trajectory_generato import TrajectoryGenerator
from mocap.msg import QuadPositionDerived

#This script provides methods that calculate a potential-based controloutput for obstacle #avoidance. The potential used is K/||x-x_o||, where x is the position of the quad and x_o is the 
#position of the obstacle. Multiple obstacles can be avoided. Which obstacles are to be avoided 
#specified in the launch file of the drone in consideration.

class AvoidanceController():
  
#In the constructor important variables are initialized and parameters are loaded. The gain can 
#be given as a parameter in the launch file. So can the array bodies. It is checked if there is 
#anything to be avoided.  
 
  def __init__(self):
    self.gain = 0.
    self.gain_z = 0.		      #z direction
    self.tg = TrajectoryGenerator()
    self.my_id = 0.
    self.bodies = []
    self.load_params()
    self.states = []
    self.obstacles_exist = True
    if len(self.bodies)>1:
      for i in range(0,len(self.bodies)):   
        self.states.append(QuadPositionDerived())
        rospy.Subscriber("/body_data/id_"+str(self.bodies[i]),QuadPositionDerived, self.__set_states)
    else:
      self.obstacles_exist = False
      self.gain = 0.

#Calculates the total potential output due to all obstacles specified in the array bodies.
  def get_potential_output(self):
    if self.obstacles_exist:
      distances = self.__get_distances()
      directions = self.__get_directions()
      u = np.array([0., 0., 0.])
      for i in range(0,len(distances)):
        if(distances[i] < 0.1):
          for j in range(0,2):
            u[j] += self.gain/0.1*directions[i][j]
            u[2] += self.gain_z/0.1 * directions[i][2] 
        else:
          for j in range(0,2):
            u[j] += self.gain/distances[i]*directions[i][j]
            u[2] += self.gain_z/distances[i]*directions[i][2]
      return u
    else:
      return [0.,0.,0.]   
    

#Returns the constant with which the potential should be blended with other controloutputs to get
#the final controloutput. This constant varies continuously from alpha_min to alpha_max. Also,
#for certain distances the value is constant at alpha_min or alpha_max respectively.
  def get_blending_constant(self):
    alpha_max = 0.8  			# <= 1.
    alpha_min = 0.			# >=0.
    r_min = 1.2 			
    r_max = 2.
    k = (alpha_min - alpha_max)/(r_max - r_min)
    m = alpha_max - k * r_min
    if self.obstacles_exist:
      distances = self.__get_distances()
      if self.gain == 0:
        return 0
      elif min(distances) <= r_min:
        return alpha_max
      elif min(distances) > r_max:
         return alpha_min
      else:
        return (k*min(distances) + m)
    else:
      return 0
    
#Updates the states when something is published on one of the mocap channels.    
  def __set_states(self,data):
    topic_name = data._connection_header["topic"]
    for i in range(0,len(self.bodies)):
      name = "/body_data/id_" + str(self.bodies[i])
      if topic_name == name:
        self.states[i] = data 

#Returns the state of the drone using this controller. 
  def __get_my_state(self):
    for i in range(0,len(self.bodies)):
      if self.bodies[i] == self.my_id:
        return self.states[i]
#Returns all distances between the drone and each object in an array. These are needed to 
#calculate the potential.    
  def __get_distances(self):
    distances = np.array([0.] * (len(self.states)-1))
    for i in range(0,len(self.states)):
      if self.bodies[i] != self.my_id:
        distance = self.tg.get_distance(self.__get_pos_from_state(self.__get_my_state()),self.__get_pos_from_state(self.states[i]))
        distances.append(distance)
    return distances

#Returns an array of all the directions from each obstacle to the drone.
  def __get_directions(self):
    directions = np.array([0.]*(len(self.states)-1))
    for i in range(0,len(self.states)):
      if self.bodies[i] != self.my_id:
        direction = self.tg.get_direction2(self.__get_pos_from_state(self.__get_my_state()),self.__get_pos_from_state(self.states[i]))
        directions.append(direction)
    return directions

#Returns the position given a certain state.
  def __get_pos_from_state(self, state):
    return [state.x,state.y,state.z]

#Loads some parameters specified in the launch file.
  def load_params(self):
    self.gain = rospy.get_param("OBSTACLE_AVOIDANCE_K",1.)
    self.bodies = rospy.get_param("OBSTACLES_TO_AVOID","[]")
    self.bodies = ast.literal_eval(self.bodies)
    self.my_id = rospy.get_param("my_id")
    
 

  
      
  















