#!/usr/bin/env python
import rospy
import ast
#import sml_setup
from controller.msg import Permission
from mocap.msg import QuadPositionDerived


# This script generates points for a follower following a leader,
# given the id of the leader and a constant offset vector between the
# two.

class Follower:  

  leader_state = QuadPositionDerived()

  def __init__(self):
    # Getting parameters
    self.offset = rospy.get_param("trajectory_generator/offset",[0.0,2.0,0])
    #self.leader_id = sml_setup.Get_Parameter('FC','trajectory_generator/leader_id',8)
    self.leader_id = rospy.get_param("trajectory_generator/leader_id",12) # change?
    if type(self.offset) is str:
      self.offset = ast.literal_eval(start_point)
    self.pub = rospy.Publisher('trajectory_gen/target',QuadPositionDerived, queue_size=10)
    self.pub_done = rospy.Publisher('trajectory_gen/done', Permission, queue_size=10)
    self.subscriber = rospy.Subscriber("body_data/id_"+str(self.leader_id),QuadPositionDerived, self.setLeaderState)
    r = 15 # Change rate to desired value
    self.rate = rospy.Rate(r)
    self.state = QuadPositionDerived()

  def follow(self):
    leader_init_state = self.leader_state
    while not rospy.is_shutdown():
      self.calculateState()
      self.pub.publish(self.state)
      self.pub_done.publish(False)
#      if self.leader_state.z <= leader_init_state: 
#        self.pub_done.publish(True)
      self.rate.sleep()

  def setLeaderState(self, data):
    self.leader_state = data
    

  # Calculates the state of the follower from the leader state
  def calculateState(self):
    self.state.x = self.leader_state.x + self.offset[0]
    self.state.y = self.leader_state.y + self.offset[1]
    self.state.z = 0.8
    self.state.yaw = self.leader_state.yaw
    self.state.x_vel = self.leader_state.x_vel
    self.state.y_vel = self.leader_state.y_vel
    self.state.z_vel = self.leader_state.z_vel
    self.state.yaw_vel = self.leader_state.yaw_vel
    self.state.x_acc = self.leader_state.x_acc
    self.state.y_acc = self.leader_state.y_acc
    self.state.z_acc = self.leader_state.z_acc
    self.state.yaw_acc = self.leader_state.yaw_acc


if __name__ == '__main__':
  rospy.init_node('follower',anonymous=True)
  try:
    follower = Follower()
    follower.follow()
  except rospy.ROSInterruptException:
    pass
