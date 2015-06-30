#!/usr/bin/env python
# Abstract follower base class

import rospy
from abc import ABCMeta, abstractmethod
from numpy import linalg as lg
from mocap.msg import QuadPositionDerived
from controller.msg import Permission
from trajectory import Trajectory
from Trajectory_node import TrajectoryNode


class Follower(Trajectory):

  __metaclass__ = ABCMeta

  leader_state = QuadPositionDerived() 
  done = False 

  def __init__(self, trajectory_node, my_id, leader_id):
    # Getting parameters
    super(Follower, self).__init__(trajectory_node)
    self.id = my_id
    self.leader_id = leader_id
    rospy.Subscriber("/body_data/id_"+str(self.leader_id),QuadPositionDerived, self.setLeaderState)
    rospy.Subscriber("/body_data/id_"+str(self.id),QuadPositionDerived, self.setMyState)
    self.calculated_state = QuadPositionDerived()
    self.real_state = QuadPositionDerived() 

    #Wait for leader
    leaderNotHere = True
    while leaderNotHere:
      leaderNotHere = not self.leader_state.found_body
    rospy.logwarn('Leader found')

  def begin():
    self.__set_done(False)

  def loop(self):
    rate = rospy.Rate(15.)
    leader_init_state = self.leader_state
    rospy.sleep(5)
    while not rospy.is_shutdown() and not self.is_done():
      self.calculateState()
      self.trajectory_node.send_msg(self.calculated_state)
      self.trajectory_node.send_permission(False)
      distance = self.__getDistance()
      rate.sleep()
      if self.leader_state.found_body == False or distance < 0.9: 
        utils.logwarn('Leader: '+str(self.leader_state.found_body)+', Distance:'+str(distance))
        self.__set_done(True)
    self.trajectory_node.send_permission(True)

  def setLeaderState(self, data):
    self.leader_state = data
    
  def setMyState(self, data):
    self.real_state = data  

  def __getDistance(self):
    my_pos = [self.real_state.x, self.real_state.y, self.real_state.z]
    leader_pos = [self.leader_state.x, self.leader_state.y, self.leader_state.z]
    temp = [0.0,0.0,0.0]
    for i in range(0,2):
      temp[i] = my_pos[i] - leader_pos[i]
    return lg.norm(temp)

  # Calculates the state of the follower from the leader state
  @abstractmethod
  def calculateState(self):
    pass

  def __set_done(self,boolean):
    self.done = boolean

  def is_done(self):
    return self.done
