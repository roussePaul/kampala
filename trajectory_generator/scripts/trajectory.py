#!/usr/bin/env python

# Class that defines the interface of a trajectory.


from abc import ABCMeta,abstractmethod


class Trajectory():
  """Class that defines the interface of a trajectory."""
  __metaclass__ = ABCMeta  

  done = False
  trajectory_node = False
  
  def __init__(self, trajectory_node):
    self.trajectory_node = trajectory_node 

  # Used for sending starting signal or similar, initial calculations, etc.
  @abstractmethod
  def begin():
    pass

  # Should containg a main loop that publishes trajectory points.
  @abstractmethod
  def loop():
    pass
  
  # A method to check if the trajectory is done, i.e. has published all points.
  def is_done(self):
    return self.done
