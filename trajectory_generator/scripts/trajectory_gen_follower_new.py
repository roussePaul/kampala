#!/usr/bin/env python
import rospy
import ast
from numpy import linalg as lg
import numpy as np
#import sml_setup
from mocap.msg import QuadPositionDerived
from controller.msg import Permission
from trajectory import Trajectory
from Trajectory_node import TrajectoryNode

#### OLD! USE trajectory_gen_follower_constant ####

# This script generates points for a follower following a leader,
# given the id of the leader and a constant offset vector between the
# two.

class Follower(Trajectory):  

  leader_state = QuadPositionDerived() 
  done = False 

  def __init__(self, trajectory_node,offset,my_id,leader_id):
    # Getting parameters
    Trajectory.__init__(self,trajectory_node)
    self.offset = offset
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
        rospy.logwarn('Leader: '+str(self.leader_state.found_body)+', Distance:'+str(distance))
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
  def calculateState(self):
    self.calculated_state.x = self.leader_state.x + self.offset[0]
    self.calculated_state.y = self.leader_state.y + self.offset[1]
    self.calculated_state.z = self.leader_state.z + self.offset[2]
    self.calculated_state.yaw = self.leader_state.yaw
    self.calculated_state.x_vel = self.leader_state.x_vel
    self.calculated_state.y_vel = self.leader_state.y_vel
    self.calculated_state.z_vel = self.leader_state.z_vel
    self.calculated_state.yaw_vel = self.leader_state.yaw_vel
    self.calculated_state.x_acc = self.leader_state.x_acc
    self.calculated_state.y_acc = self.leader_state.y_acc
    self.calculated_state.z_acc = self.leader_state.z_acc
    self.calculated_state.yaw_acc = self.leader_state.yaw_acc

  def __set_done(self,boolean):
    self.done = boolean

  def is_done(self):
    return self.done



