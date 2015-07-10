#!/usr/bin/env python
import rospy
import sys
import ast
import math
import numpy
import trajectory_generator
from trajectory_generato import TrajectoryGenerator
from mocap.msg import QuadPositionDerived

class AvoidanceController():
  
  
 
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
      
  def get_potential_output(self):
    if self.obstacles_exist:
      distances = self.__get_distances()
      directions = self.__get_directions()
      u = [0., 0., 0.]
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
    
    
  def __set_states(self,data):
    topic_name = data._connection_header["topic"]
    for i in range(0,len(self.bodies)):
      name = "/body_data/id_" + str(self.bodies[i])
      if topic_name == name:
        self.states[i] = data 
  
  def __get_my_state(self):
    for i in range(0,len(self.bodies)):
      if self.bodies[i] == self.my_id:
        return self.states[i]
    
  def __get_distances(self):
    distances = []
    for i in range(0,len(self.states)):
      if self.bodies[i] != self.my_id:
        distance = self.tg.get_distance(self.__get_pos_from_state(self.__get_my_state()),self.__get_pos_from_state(self.states[i]))
        distances.append(distance)
    return distances

  def __get_directions(self):
    directions = []
    for i in range(0,len(self.states)):
      if self.bodies[i] != self.my_id:
        direction = self.tg.get_direction2(self.__get_pos_from_state(self.__get_my_state()),self.__get_pos_from_state(self.states[i]))
        directions.append(direction)
    return directions


  def __get_pos_from_state(self, state):
    return [state.x,state.y,state.z]

  def load_params(self):
    self.gain = rospy.get_param("OBSTACLE_AVOIDANCE_K",1.)
    self.bodies = rospy.get_param("OBSTACLES_TO_AVOID",[])
    self.my_id = rospy.get_param("my_id")
    
 
  #def __get_acceleration(self):
    #distances = self.__get_distances()
    #directions = self.__get_directions()
    #alpha = 0.1
    #r_0  = 10
    #acc = [0.,0.,0.]
    #for j in range(0,len(self.states)):
     # for i in range(0,2):
    #    acc[i] += alpha/r_0 * math.e**(-distances[j]/r_0) * directions[j][i] 
   # return acc

  #def __get_velocity(self, current_velocity, acceleration,dt):
    #velocity = [0.,0.,0.]
    #for i in range(0,2):
     # velocity[i] = current_velocity[i] + acceleration[i]*dt
    #return velocity
  
  #def __get_position(self, current_position,velocity, dt):
   # position = [0.,0.,0.]
    #for i in range(0,2):
     # position[i] = current_position[i] + velocity[i] *dt    
    #return position

  #def get_reference(self,dt):
   # my_state = self.__get_my_state()
    #acc = self.__get_acceleration()
    #vel = self.__get_velocity([my_state.x_vel,my_state.y_vel,my_state.z_vel], acc, dt)
    #pos = self.__get_position([my_state.x,my_state.y,my_state.z],vel,dt)
    #acc.append(0)
    #vel.append(0)
    #pos.append(0)
    #ref = self.tg.get_message(pos,vel,acc)
    #return ref

  
      
  















