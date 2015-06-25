#!/usr/bin/env python
import rospy
import ast
from numpy import linalg as lg
import numpy as np
#import sml_setup
from mocap.msg import QuadPositionDerived
from controller.msg import Permission


# This script generates points for a follower following a leader,
# given the id of the leader and a constant offset vector between the
# two.

class Follower:  

  leader_state = QuadPositionDerived()  

  def __init__(self):
    # Getting parameters
    self.offset = rospy.get_param("/trajectory_generator/offset",[0.0,3.0,0])
    self.id = rospy.get_param('/trajectory_generator/follower_id',16)
    self.leader_id = rospy.get_param("/trajectory_generator/leader_id",8) # change?
    if type(self.offset) is str:
      self.offset = ast.literal_eval(start_point)
    self.pub = rospy.Publisher('trajectory_gen/target',QuadPositionDerived, queue_size=10)
    self.pub_done = rospy.Publisher('trajectory_gen/done', Permission, queue_size=10)
    rospy.Subscriber("/body_data/id_"+str(self.leader_id),QuadPositionDerived, self.setLeaderState)
    rospy.Subscriber("/body_data/id_"+str(self.id),QuadPositionDerived, self.setMyState)
    r = 15 # Change rate to desired value
    self.rate = rospy.Rate(r)
    self.calculated_state = QuadPositionDerived()
    self.real_state = QuadPositionDerived() 

    #Wait for leader
    leaderNotHere = True
    while leaderNotHere:
      leaderNotHere = not self.leader_state.found_body
    rospy.logwarn('Leader found')

   
  def follow(self):
    leader_init_state = self.leader_state
    rospy.sleep(5)
    while not rospy.is_shutdown():
      self.calculateState()
      self.pub.publish(self.calculated_state)
      distance = self.__getDistance()

      if self.leader_state.found_body == False or distance < 0.9: 
        rospy.logwarn('Leader: '+str(self.leader_state.found_body)+', Distance:'+str(distance))
        self.pub_done.publish(True)
      else:
        self.pub_done.publish(False)

      self.rate.sleep()

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


if __name__ == '__main__':
  rospy.init_node('follower',anonymous=True)
  try:
    follower = Follower()
    follower.follow()
  except rospy.ROSInterruptException:
    pass
