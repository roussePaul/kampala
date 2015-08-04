#!/usr/bin/env python


import rospy
import sml_setup
import sys
import math
from controller_base import Controller
from point import *
from controller.msg import PlotData
from mavros.msg import OverrideRCIn
from mocap.msg import QuadPositionDerived
from std_srvs.srv import Empty
import analysis
import utils
from numpy import linalg as lg
import numpy as np
from std_msgs.msg import Float64
import copy


class PID(Controller):
  """This class provides the PID controller used for trajectory tracking with the Iris+."""
  
  def __init__(self):
    """In the constructor the parameters for the drone in consideration are loaded.
    The integral action is initialized to zero and a service for updating the parameters 
    during flight is provided (PID_controller/update_parameters)."""
    self.load_PID_parameters()
    self.d_updated = np.array([0.,0.,0.]) # Integral term
    self.integral_xy_pub = rospy.Publisher('visualization/integral_xy',Float64,queue_size = 10) 
    self.integral_z_pub = rospy.Publisher('visualization/integral_z',Float64,queue_size = 10)
    rospy.Service('PID_controller/update_parameters', Empty, self.update_parameters)
  

  def reset(self):
    utils.logwarn("PID reset")
    """This function resets the integral action."""
    self.d_updated = np.array([0.,0.,0.])

  ##@return the current value of the integral action
  def get_d_updated(self):
    return self.d_updated

  ##@param d: a numpy.array to set the integral action to
  def set_d_updated(self, d):
    self.d_updated = copy.deepcopy(d)

  ##@param current: a list of 3 elements representing the current position, velocity etc.
  ##@param target: a list of 3 elements representing the target position, velocity etc.
  ##@return the elementwise difference current - target
  def get_errors(self,current,target):
    e=[]
    for i in range (0,3):
      e.append(current[i]-target[i])

    return e

  ##@param current: a point object specifying the current state of the drone
  ##@param target: a point object specifying the target of the drone
  ##@return the acceleration calculated from the PID
  def get_output(self,current,target):
    d_updated = copy.deepcopy(self.get_d_updated())
    x,x_vel,x_acc=get_pos_vel_acc(current)  # current state
    x_target,x_vel_target,x_acc_target=get_pos_vel_acc(target) # target state
    time_diff=current.time_diff 
    command_controlled=self.calculate_PID_output(x,x_vel,x_target,x_vel_target,x_acc_target,time_diff,d_updated)
    return command_controlled

  ##@param msg: the message created by the service PID_controller/update_parameters
  def update_parameters(self,msg):
    """This function is invoked when the service PID_controller/update_parameters is called.
    It notifies that the parameters have been loaded and calls the function that loads the parameters."""
    utils.loginfo('PID parameters loaded')
    self.load_PID_parameters()
    return []

  ##@param x: a list containing the current position of the drone
  ##@param x_vel: a list containing the current velocity of the drone
  ##@param x_target: a list containing the target position of the drone
  ##@param x_vel_target: a list containing the target velocity of the drone
  ##@param x_acc_target: a list containing the target acceleration of the drone
  ##@param delta_t: the time difference between the last update of the current state and the most recent one
  ##@param current_d: the current value of the integral action
  ##@return the accelerations calculated from the errors using the PID
  def calculate_PID_output(self,x,x_vel,x_target,x_vel_target,x_acc_target,delta_t,current_d):
    u=np.array([0.]*3)
    acc_target = np.array(x_acc_target)
    new_d = np.array([0.]*3)
    #Compute errors

    e=np.array(self.get_errors(x,x_target))

    e_dot=np.array(self.get_errors(x_vel,x_vel_target))
    for i in range(0,3):
      new_d[i]=current_d[i]+delta_t*(self.K_i[i]*((e[i]*self.Kv[i]/2)+e_dot[i]))
      new_d[i]=self.saturation(new_d[i],-self.I_lim[i],self.I_lim[i])
      u_p = self.saturation(self.Kp[i]*e[i], -self.Max_acc, self.Max_acc)
      u_v = self.saturation(self.Kv[i]*e_dot[i], -self.Max_acc, self.Max_acc)
      u[i] = acc_target[i]-u_v-u_p
      u[i] = u[i] - new_d[i]
    self.set_d_updated(new_d)
    self.integral_xy_pub.publish(new_d[0])
    self.integral_z_pub.publish(new_d[2])   
    return u

  ##@param value: a real number
  ##@param minimum: the minimum allowed value of value
  ##@param maximum: the maximum allowed value of value
  ##@return: value if minimum <= value <= maximum, minimum if value < minimum < maximum, maximum if minimum < maximum < value 
  def saturation(self,value,minimum,maximum):
    value=max(minimum,min(maximum,value))
    return value
	

  def load_PID_parameters(self):
    """This function is used to load the parameters of the PID specified in the launch file
    of the drone."""		
    #Controller parameters	
    self.w = utils.Get_Parameter("PID_w",1.7)
    self.w_z  = utils.Get_Parameter("PID_w_z", 1.3)
    self.x_i = utils.Get_Parameter("PID_x_i",0.7)

    Kp = utils.Get_Parameter("PID_Kp",self.w*self.w)
    Kv = utils.Get_Parameter("PID_Kv",2*self.x_i*self.w)

    Kp_z = utils.Get_Parameter("PID_Kp_z", self.w_z*self.w_z)
    Kv_z = utils.Get_Parameter("PID_Kv_z", 2*self.x_i*self.w_z)

    I_lim = utils.Get_Parameter("PID_I_lim",0.5)
    K_i = utils.Get_Parameter("PID_K_i",7)
    I_lim_z = utils.Get_Parameter("PID_I_lim_z",0.5)
    K_i_z = utils.Get_Parameter("PID_K_i_z",7)

    self.Max_acc = utils.Get_Parameter("PID_Max_acc",1.0) 

    self.R = utils.Get_Parameter("PID_R",1.0) 
    self.Kp = np.array([Kp,Kp,Kp_z])
    self.Kv = np.array([Kv,Kv,Kv_z])
    self.K_i = np.array([K_i,K_i,K_i_z])
    self.I_lim = np.array([I_lim,I_lim,I_lim_z])

    self.reset()

#EOF
