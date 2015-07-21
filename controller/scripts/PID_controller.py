#!/usr/bin/env python

# PID controller for the IRIS+ in the SML Lab 
# Gets inputs from the Security Guard and the Trajectory Generator
# Publishes commands via Mavros' rc/override topic


import rospy
import sml_setup
import sys
import math
from controller_base import Controller
from point import *
from controller.msg import PlotData
from mavros.msg import OverrideRCIn
from mocap.msg import QuadPositionDerived
from controller.msg import Permission
from std_srvs.srv import Empty
import analysis
import utils
from numpy import linalg as lg
import numpy as np
import copy

#Constants
#*************************************
NODE_NAME='PID'
#*************************************


class PID(Controller):
  def __init__(self):
    self.load_PID_parameters()
    self.d_updated = np.array([0.,0.,0.]) # Integral term
    rospy.Service('PID_controller/update_parameters', Empty, self.update_parameters)
  

  # Resets PID
  def reset(self):
    self.d_updated = np.array([0.,0.,0.])


  def get_d_updated(self):
    return self.d_updated


  def set_d_updated(self, d):
    self.d_updated = copy.deepcopy(d)


  def get_errors(self,current,target):
    e=[]
    for i in range (0,3):
      e.append(current[i]-target[i])

    return e

  # Returns the output of the controller
  def get_output(self,current,target):
    d_updated = copy.deepcopy(self.get_d_updated())
    x,x_vel,x_acc=get_pos_vel_acc(current)  # current state
    x_target,x_vel_target,x_acc_target=get_pos_vel_acc(target) # target state
    time_diff=current.time_diff 
    command_controlled=self.calculate_PID_output(x,x_vel,x_acc,x_target,x_vel_target,x_acc_target,time_diff,d_updated)
    return command_controlled


  def update_parameters(self,msg):
    utils.loginfo('PID parameters loaded')
    self.load_PID_parameters()
    return []


  def calculate_PID_output(self,x,x_vel,x_acc,x_target,x_vel_target,x_acc_target,delta_t,current_d):
    u=np.array([0.]*3)
    acc_target = np.array(x_acc_target)
    new_d = np.array([0.]*3)
    #Compute errors
    e=np.array(self.get_errors(x,x_target))
    e_dot=np.array(self.get_errors(x_vel,x_vel_target))
    for i in range(0,3):
      new_d[i]=current_d[i]+delta_t*(self.K_i[i]*((e[i]*self.Kv[i]/2)+e_dot[i]))
      new_d[i]=self.saturation(new_d[i],-self.I_lim[i],self.I_lim[i])
      u[i] = acc_target[i]-self.Kv[i]*e_dot[i]-self.Kp[i]*e[i]
      u[i] = u[i] - new_d[i]
    self.set_d_updated(new_d)   
    return u


  def saturation(self,value,minimum,maximum):
    value=max(minimum,min(maximum,value))
    return value
	

  # Read parameters for PID
  # The parameters are specified in the launch file of each drone.
  def load_PID_parameters(self):		
    #Controller parameters	
    self.w = sml_setup.Get_Parameter(NODE_NAME,"PID_w",1.5)
    self.w_z  = sml_setup.Get_Parameter(NODE_NAME,"PID_w_z", 0.5)
    self.x_i = sml_setup.Get_Parameter(NODE_NAME,"PID_x_i",0.7)
    Kp = sml_setup.Get_Parameter(NODE_NAME,"PID_Kp",self.w*self.w)
    Kv = sml_setup.Get_Parameter(NODE_NAME,"PID_Kv",2*self.x_i*self.w)

    Kv_z = sml_setup.Get_Parameter(NODE_NAME,"PID_Kv_z", 2*self.x_i*self.w_z)
    Kp_z = sml_setup.Get_Parameter(NODE_NAME,"PID_Kp_z", self.w_z*self.w_z)

    I_lim = sml_setup.Get_Parameter(NODE_NAME,"PID_I_lim",0.5)
    K_i = sml_setup.Get_Parameter(NODE_NAME,"PID_K_i",1.)
    I_lim_z = sml_setup.Get_Parameter(NODE_NAME,"PID_I_lim_z",0.5)
    K_i_z = sml_setup.Get_Parameter(NODE_NAME,"PID_K_i_z",1.)
    self.Kp = np.array([Kp,Kp,Kp_z])
    self.Kv = np.array([Kv,Kv,Kv_z])
    self.K_i = np.array([K_i,K_i,K_i_z])
    self.I_lim = np.array([I_lim,I_lim,I_lim_z])

#EOF
