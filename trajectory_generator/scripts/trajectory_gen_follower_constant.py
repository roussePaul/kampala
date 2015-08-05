#!/usr/bin/env python



import rospy
from trajectory_gen_follower import Follower
from trajectory_node import TrajectoryNode


class ConstantFollower(Follower):  
  """This class generates points for a follower following a leader,
  given the id of the leader and a constant offset vector between the
  two."""

  def __init__(self, trajectory_node, offset, my_id, leader_id):
    self.__offset = offset
    # The loop operates in the base class
    super(ConstantFollower, self).__init__(trajectory_node, my_id, leader_id)

  # Calculates the state of the follower from the leader state
  def calculate_state(self):
    self.calculated_state.x = self.leader_state.x + self.__offset[0] 
    self.calculated_state.y = self.leader_state.y + self.__offset[1]
    self.calculated_state.z = self.leader_state.z + self.__offset[2]
    #self.calculated_state.yaw = self.leader_state.yaw
    self.calculated_state.x_vel = self.leader_state.x_vel
    self.calculated_state.y_vel = self.leader_state.y_vel
    self.calculated_state.z_vel = self.leader_state.z_vel
    #self.calculated_state.yaw_vel = self.leader_state.yaw_vel
    #self.calculated_state.x_acc = self.leader_state.x_acc
    #self.calculated_state.y_acc = self.leader_state.y_acc
    #self.calculated_state.z_acc = self.leader_state.z_acc
    #self.calculated_state.yaw_acc = self.leader_state.yaw_acc


if __name__ == '__main__':
  try:
    traj = TrajectoryNode()
    offset = [0.0,-1.0,0.0]
    my_id = 21
    leader_id = 8
    ConstantFollower(traj,offset,my_id,leader_id).loop()
  except rospy.ROSInterruptException:
    pass
