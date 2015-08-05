#!/usr/bin/env python



import rospy
from abc import ABCMeta, abstractmethod
from numpy import linalg as lg
from mocap.msg import QuadPositionDerived
from controller.msg import Permission
from trajectory import Trajectory
from trajectory_node import TrajectoryNode

class Follower(Trajectory):
  """This is an abstract follower base class."""
  __metaclass__ = ABCMeta
  
  ##@param leader_state: the state of the leader
  leader_state = QuadPositionDerived() 
  #@param done: tells whether or not the trajectory is done
  done = False 

  def __init__(self, trajectory_node, my_id, leader_id):
    # Getting parameters
    super(Follower, self).__init__(trajectory_node)
    self.__id = my_id
    self.__leader_id = leader_id
    rospy.Subscriber("/body_data/id_"+str(self.__leader_id),QuadPositionDerived, self.__set_leader_state)
    rospy.Subscriber("/body_data/id_"+str(self.__id),QuadPositionDerived, self.__set_my_state)
    self.calculated_state = QuadPositionDerived()
    self.__real_state = QuadPositionDerived() 

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
    rospy.sleep(5.)

    # Main loop for publishing points.
    while not rospy.is_shutdown() and not self.is_done():
      self.calculate_state()
      self.trajectory_node.send_msg(self.calculated_state)
      self.trajectory_node.send_permission(False)
      distance = self.__getDistance()
      rate.sleep()
      if self.leader_state.found_body == False or distance < 0.5: 
        #utils.logwarn('Leader: '+str(self.leader_state.found_body)+', Distance:'+str(distance))
        self.__set_done(True)
    self.trajectory_node.send_permission(True)

  # Sets state of leader.
  ##@param data: the mocap data to set the leader state to.
  def __set_leader_state(self, data):
    self.leader_state = data
    
  # Sets the state of follower - the quadcopter for which the trajectory is generated. 
  ##@param data: the mocap data to set the follower state to.
  def __set_my_state(self, data):
    self.__real_state = data  

  # Calculates distance between leader and follower.
  ##@return the distance between leader and follower
  def __getDistance(self):
    my_pos = [self.__real_state.x, self.__real_state.y, self.__real_state.z]
    leader_pos = [self.leader_state.x, self.leader_state.y, self.leader_state.z]
    temp = [0.0,0.0,0.0]
    for i in range(0,3):
      temp[i] = my_pos[i] - leader_pos[i]
    return lg.norm(temp)


  @abstractmethod
  def calculate_state(self):
    """This function should calculate the state of the follower from the leader state."""
    pass

  def __set_done(self,boolean):
    self.done = boolean

  def is_done(self):
    return self.done
