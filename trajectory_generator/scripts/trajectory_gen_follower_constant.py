#!/usr/bin/env python



import rospy
from trajectory_gen_follower import Follower
from Trajectory_node import TrajectoryNode

class ConstantFollower(Follower):  
  """This class generates points for a follower following a leader,
  given the id of the leader and a constant offset vector between the
  two."""

  def __init__(self, trajectory_node, offset, my_id, leader_id):
    self.offset = offset
    # The loop operates in the base class
    super(ConstantFollower, self).__init__(trajectory_node, my_id, leader_id)

  # Calculates the state of the follower from the leader state
  def calculateState(self):
    self.calculated_state.x = self.leader_state.x + self.offset[0]
    self.calculated_state.y = self.leader_state.y + self.offset[1]
    self.calculated_state.z = self.leader_state.z + self.offset[2]
    #self.calculated_state.yaw = self.leader_state.yaw
    self.calculated_state.x_vel = self.leader_state.x_vel
    self.calculated_state.y_vel = self.leader_state.y_vel
    self.calculated_state.z_vel = self.leader_state.z_vel
    #self.calculated_state.yaw_vel = self.leader_state.yaw_vel
    #self.calculated_state.x_acc = self.leader_state.x_acc
    #self.calculated_state.y_acc = self.leader_state.y_acc
    #self.calculated_state.z_acc = self.leader_state.z_acc
    #self.calculated_state.yaw_acc = self.leader_state.yaw_acc



