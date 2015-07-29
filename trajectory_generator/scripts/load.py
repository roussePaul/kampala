#!/usr/bin/env python



import rospy
from numpy import array, sin,  cos
from mocap.msg import QuadPositionDerivedExt
from trajectory_generato import TrajectoryGenerator
from trajectory import Trajectory
from Trajectory_node import TrajectoryNode
from Trajectory_node_ext import TrajectoryNodeExt
from straight_line_class import StraightLineGen


class LoadTrajectory(Trajectory):
  """This script generates the points, velocities, accelerations, jerk
  and snap to be used as a reference for the load, for the load
  transport controller."""
  
  done = False
  a_max = 0.6**2.0/0.8
  
  def __init__(self,trajectory_node):
    Trajectory.__init__(self,trajectory_node)

  
  def begin(self):
    self.__set_done(False)

  # Desired trajectory for LOAD
  def traj_des(self, t):
    r = 1.0
    w = 0.5
    
    p = r*w**0*array([ cos(w*t), sin(w*t),0.0]) + array([0,0,1.0]);
    v = r*w**1*array([-sin(w*t), cos(w*t),0.0]);
    a = r*w**2*array([-cos(w*t),-sin(w*t),0.0]);
    j = r*w**3*array([ sin(w*t),-cos(w*t),0.0]);
    s = r*w**4*array([ cos(w*t), sin(w*t),0.0]);

    #p = array([0,2.5,-4.8]);
    #v = array([0,0,0]);
    #a = array([0,0,0]);
    #j = array([0,0,0]);
    #s = array([0,0,0]);

    msg = QuadPositionDerivedExt()
    msg.x = p[0]; msg.y = p[1]; msg.z = p[2]
    msg.x_vel = v[0]; msg.y_vel = v[1]; msg.z_vel = v[2]
    msg.x_acc = a[0]; msg.y_acc = a[1]; msg.z_acc = a[2]
    msg.x_jerk = j[0]; msg.y_jerk = j[1]; msg.z_jerk = j[2]
    msg.x_snap = s[0]; msg.y_snap = s[1]; msg.z_snap = s[2]
    
    return msg
    

  def loop(self, start_time):
    time = start_time
    r = 10.0
    rate = rospy.Rate(r)
    while not rospy.is_shutdown() and not self.is_done():
      outmsg = self.traj_des(time)      
      self.trajectory_node.send_msg(outmsg)
      self.trajectory_node.send_permission(False)
      rate.sleep()
      time += 1/r

    
  def is_done(self):
    return self.done

  def __set_done(self,boolean):
    self.done = boolean
   
if __name__ == '__main__':
  try:
    rospy.sleep(4.)
    
    # Start and hover
    #trajectory_node_line = TrajectoryNode()
    #sl_gen = StraightLineGen(trajectory_node_line, [0.,0.,0.2], [0.,0.,0.6])
    #sl_gen.loop(0.)
    #rospy.sleep(2.)

    # Go to and pickup load
    
    # Fly load (now a circle)
    trajectory_node = TrajectoryNodeExt()
    load_trajectory = LoadTrajectory(trajectory_node)
    load_trajectory.loop(0.)

  except rospy.ROSInterruptException:
    pass
  
