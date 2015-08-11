#!/usr/bin/env python
import rospy
import sys
import ast
import math
import numpy as np
import trajectory_generator
from trajectory_generato import TrajectoryGenerator
from mocap.msg import QuadPositionDerived
from std_srvs.srv import Empty
import utils

## This script provides methods that calculate a potential-based controloutput for obstacle 
## avoidance. The potential used is \f$\frac{K}{\left \| x-x_0 \right \|}\f$, where \f$x\f$ is the position of the quad and \f$x_0\f$ is the 
## position of the obstacle. Multiple obstacles can be avoided. Which obstacles are to be avoided 
## specified in the launch file of the drone in consideration.
class AvoidanceController():
  ## In the constructor important variables are initialized and parameters are loaded. The gain can 
  ## be given as a parameter in the launch file. So can the array bodies. It is checked if there is 
  ## anything to be avoided.  
  def __init__(self):
    self.gain = 0.		      
    self.tg = TrajectoryGenerator()
    rospy.Service('obstacle_avoidance/update_parameters', Empty, self.update_parameters)
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


  ##@return the potential calculated from all obstacles
  def get_potential_output(self):
    """This function calculates the total potential output due to all obstacles specified in the array bodies.
    Observe that bodies that are not detected are ignored as they are assumed to be out of the 
    tracking area of the motion capture system. As soon as the obstacle is tracked by the motion 
    capture it adds a contribution to the potential."""
    if self.obstacles_exist and self.obstacles_detected():
      distances = self.__get_distances()
      directions = self.__get_directions()
      u = [0., 0., 0.]
      for i in range(0,len(distances)):
        if(distances[i] < 0.1):
          for j in range(0,2):
            u[j] += self.gain/0.1*directions[i][j]
        else:
          for j in range(0,2):
            u[j] += self.gain/distances[i]*directions[i][j]
      return np.array(u)
    else:
      return np.array([0.,0.,0.])   
    

  ##@return the constant with which the potential should be blended with other control outputs
  def get_blending_constant(self):
    """This function returns the constant with which the potential should be blended with other control outputs to get
    the final control output. This constant varies continuously from alpha_min to alpha_max. Also,
    for certain distances the value is constant at alpha_min or alpha_max respectively. If none of 
    the obstacles is detected, the constant is set to zero."""
    alpha_max = 0.6  			# <= 1. iris4 0.65
    alpha_min = 0.2			# >=0.
    r_min = 0.9 			
    r_max = 2.
    k = (alpha_min - alpha_max)/(r_max - r_min)
    m = alpha_max - k * r_min
    if self.obstacles_exist and self.obstacles_detected():
      distances = self.__get_distances()
      if self.gain == 0:
        return 0.
      elif min(distances) <= r_min:
        return alpha_max
      elif min(distances) > r_max:
         return alpha_min
      else:
        return (k*min(distances) + m)
    else:
      return 0.
    
  def __set_states(self,data):
    """This function updates the states when something is published on one of the mocap topics."""    
    topic_name = data._connection_header["topic"]
    for i in range(0,len(self.bodies)):
      name = "/body_data/id_" + str(self.bodies[i])
      if topic_name == name:
        self.states[i] = data 

  ##@return the state of the drone using this controller
  def __get_my_state(self):
    for i in range(0,len(self.bodies)):
      if self.bodies[i] == self.my_id:
        return self.states[i]

  ##@return true if at least one of the specified obstacles has been found, false otherwise
  def obstacles_detected(self):
    detected = False
    for i in range(0,len(self.states)):
      if self.states[i].found_body and self.bodies[i] != self.my_id:
        detected = True
        break
    return detected   

  ## Returns all distances between the drone and each object in an array. By distance, the projection of the
  ## distance on the xy-plane is meant. These are needed to calculate the potential.    
  ##@return a list containing all distances between the drone and each object it should avoid. y distance, the projection of the distance on the xy-plane is meant.    
  def __get_distances(self):
    distances = []
    for i in range(0,len(self.states)):
      if self.bodies[i] != self.my_id and self.states[i].found_body:
        my_pos = self.__get_pos_from_state(self.__get_my_state())
        obstacle_pos = self.__get_pos_from_state(self.states[i])
        vector = self.tg.get_vector(my_pos,obstacle_pos)
        x_proj = self.tg.get_projection(vector,[1.,0.,0.])
        y_proj = self.tg.get_projection(vector,[0.,1.,0.])
        distance = self.tg.get_norm([x_proj,y_proj,0.])
        distances.append(distance)
    return distances

  ## Returns an array of all the directions from each obstacle to the drone. Observe that these directions are 
  ## radially outward in a coordinate system, which has its z-axis through the obstacle. No potential is
  ## generated along the z-axis and the potential is independent of the z-direction.
  ##@return a list of the projections of the directions from each obstacle to the drone on the xy-plane
  def __get_directions(self):
    directions = []
    for i in range(0,len(self.states)):
      if self.bodies[i] != self.my_id and self.states[i].found_body:
        direction = self.tg.get_direction2(self.__get_pos_from_state(self.__get_my_state()),self.__get_pos_from_state(self.states[i]))
        x_proj = self.tg.get_projection(direction,[1.,0.,0.])
        y_proj = self.tg.get_projection(direction,[0.,1.,0.])
        direction = self.tg.get_direction([x_proj,y_proj,0.])
        directions.append(direction) 
    return directions

  ## Returns the position given a certain state.
  ##@param state: the state of an obstacle in form of a QuadPositionDerived
  ##@return a list of the form [state.x,state.y,state.z]
  def __get_pos_from_state(self, state):
    return [state.x,state.y,state.z]

  def load_params(self):
    """This function loads some parameters specified in the launch file. These are the gain of the
    potential, the bodies to avoid and the id of the drone itself."""
    self.gain = rospy.get_param("OBSTACLE_AVOIDANCE_K",1.)
    self.bodies = rospy.get_param("OBSTACLES_TO_AVOID","[2]")
    self.bodies = ast.literal_eval(self.bodies)
    self.my_id = rospy.get_param("my_id")
    self.bodies.append(self.my_id)
  
  ##@param msg: the message created when calling the service obstacle_avoidance/update_parameters     
  def update_parameters(self,msg):
    """This is the function that is invoked when the service obstacle_avoidance/update_parameters is called.
    It notifies that the parameters are loaded and calls load_params."""
    utils.loginfo('Obstacle avoidance parameters loaded')
    self.load_params()
    return[]















