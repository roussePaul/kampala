#!/usr/bin/env python

# This script generates points for a follower following a leader,
# given the id of the leader and a constant offset vector between the
# two.

import rospy
from trajectory_gen_follower import Follower
from trajectory_node import TrajectoryNode

class ConstantFollower(Follower):  

  def __init__(self, trajectory_node, offset, my_id, leader_id):
    self.offset = offset
    # The loop operates in the base class
    super(ConstantFollower, self).__init__(trajectory_node, my_id, leader_id)

  # Calculates the state of the follower from the leader state
  def calculate_state(self):
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


if __name__ == '__main__':
  try:
    #my_id = rospy.get_param("/trajectory_generator/follower_id")
    #leader_id = rospy.get_param("/trajectory_generator/leader_id")
    follower = ConstantFollower(TrajectoryNode(),[0.,2.,0.],8,20)
    follower.loop()
  except rospy.ROSInterruptException:
    pass
