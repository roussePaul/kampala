#!/usr/bin/env python
import rospy
import sys
import ast
import math
import numpy
from mocap.msg import QuadPositionDerived
from trajectory_generato import TrajectoryGenerator
from trajectory import Trajectory
from Trajectory_node import TrajectoryNode
from straight_line_class import StraightLineGen
from circle_acc import AccGen
from hover_and_hook import Hooker
from std_srvs.srv import Empty



class HookDemo():
  """This script is for demonstrating the ability of Iris to "hook" an object."""
   
  done = False
  
  def __init__(self,load_id, goal):
    self.__node = TrajectoryNode()
    self.tg = TrajectoryGenerator()
    self.__name = rospy.get_param('my_name')
    self.__goal = goal
    self.__state = QuadPositionDerived()
    self.__load_pos = QuadPositionDerived()
    rospy.Subscriber('/body_data/id_'+str(load_id),QuadPositionDerived,self.__set_load_pos)
    my_id = rospy.get_param('my_id')
    rospy.Subscriber('/body_data/id_'+str(my_id),QuadPositionDerived,self.__set_state)
    rospy.sleep(2.)

    
  def __set_state(self,data):
    self.__state = data

  def __set_load_pos(self,data):
    self.__load_pos = data

  def __adjust_gravity_cancel(self,incr):
        current = rospy.get_param('/' + self.__name + '/CONTROL_CANCEL_GRAVITY')
        new = current + incr
        rospy.set_param('/' + self.__name + '/CONTROL_CANCEL_GRAVITY',new)
        try: 
            params_load = rospy.ServiceProxy("/%s/blender/update_parameters"%(self.__name), Empty)
            params_load_PID = rospy.ServiceProxy("/%s/PID_controller/update_parameters"%(self.__name), Empty)
            params_load()
            params_load_PID()
        except rospy.ServiceException as exc:
            utils.loginfo("PID not reachable " + str(exc))

  def demo(self):
    """Maneuvers the drone over the load, lowers it, hooks and moves to the goal."""
    start = [self.__state.x,self.__state.y,self.__state.z]
    end = [self.__state.x,self.__state.y,1.]
    StraightLineGen(self.__node,start,end).loop(0.)
    rospy.sleep(2.)
    start = [self.__state.x,self.__state.y,self.__state.z]
    end = [self.__load_pos.x,self.__load_pos.y,1.]
    StraightLineGen(self.__node,start,end).loop(0.)
    rospy.sleep(4.)
    start = [self.__state.x,self.__state.y,self.__state.z]
    end = [self.__load_pos.x-0.02,self.__load_pos.y+0.01,0.9]
    StraightLineGen(self.__node,start,end).loop(0.)
    rospy.sleep(2.)
    start = [self.__state.x,self.__state.y,self.__state.z]
    end = [self.__load_pos.x-0.02,self.__load_pos.y+0.01,0.85]
    StraightLineGen(self.__node,start,end).loop(0.)
    hooker = Hooker(self.__node)
    hooker.hover_and_hook(end)
    self.__adjust_gravity_cancel(50)
    rospy.sleep(0.1)
    dist = self.__state.z - self.__load_pos.z
    dist = 0.2
    if dist > 0.4:
      self.__adjust_gravity_cancel(-50)
      start = [self.__state.x,self.__state.y,self.__state.z]
      end = [self.__state.x,self.__state.y,1.]
      StraightLineGen(self.__node,start,end).loop(0.)
      # maybe try again?
      rospy.sleep(2.)
      start = [self.__state.x,self.__state.y,self.__state.z]
      end = [0.,0.,1.]
      StraightLineGen(self.__node,start,end).loop(0.)
    else:
      start = [self.__state.x,self.__state.y,self.__state.z]
      end = [self.__state.x,self.__state.y,self.__state.z+0.2]
      StraightLineGen(self.__node,start,end).loop(0.)
      rospy.sleep(0.2)
      dist = self.__state.z - self.__load_pos.z
      dist = 0.2
      if dist > 0.4:
        self.__adjust_gravity_cancel(-50)
        start = [self.__state.x,self.__state.y,self.__state.z]
        end = [self.__state.x,self.__state.y,1.]
        StraightLineGen(self.__node,start,end).loop(0.)
        rospy.sleep(2.)
        start = [self.__state.x,self.__state.y,self.__state.z]
        #maybe try again?
        end = [0.,0.,1.]
        StraightLineGen(self.__node,start,end).loop(0.)
      else:
        start = [self.__state.x,self.__state.y,self.__state.z]
        end = [self.__state.x,self.__state.y,1.]
        StraightLineGen(self.__node,start,end).loop(0.)
        rospy.sleep(2.)
        start = [self.__state.x,self.__state.y,self.__state.z]
        end = [self.__goal[0],self.__goal[1],1.]
        StraightLineGen(self.__node,start,end).loop(0.)
        rospy.sleep(2.)
        start = [self.__state.x,self.__state.y,self.__state.z]
        end = [self.__goal[0],self.__goal[1],0.85]
        StraightLineGen(self.__node,start,end).loop(0.)
        rospy.sleep(2.)
        hooker.hover_and_unhook(self.__goal)
        self.__adjust_gravity_cancel(-50)
        current_z = self.__state.z
        start = [self.__state.x,self.__state.y,self.__state.z]
        end = [self.__state.x,self.__state.y,self.__state.z+0.25]
        StraightLineGen(self.__node,start,end).loop(0.)
        rospy.sleep(0.1)
        new_z = self.__state.z
        dist = new_z - current_z
        if dist < 0.1:
          self.__adjust_gravity_cancel(50)
        start = [self.__state.x,self.__state.y,self.__state.z]
        end = [0.,-1.,1.]
        StraightLineGen(self.__node,start,end).loop(0.)
        
       
        
        
        
     
    
  
if __name__ == '__main__':
  try:
    hook_demo = HookDemo(2,[0.,0.,1.])
    hook_demo.demo()
  except rospy.ROSInterruptException:
    pass
  
