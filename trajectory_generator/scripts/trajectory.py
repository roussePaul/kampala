#!/usr/bin/env python
from abc import ABCMeta,abstractmethod
#Class that defines the interface of a trajectory

class Trajectory():

  __metaclass__ = ABCMeta  

  done = False
  trajectory_node = False
  
  def __init__(self, trajectory_node):
    self.trajectory_node = trajectory_node 

  @abstractmethod
  def begin():
    pass

  @abstractmethod
  def loop():
    pass
  
  
  def is_done(self):
    return self.done
