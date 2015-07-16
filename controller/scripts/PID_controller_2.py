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

from pid import pid_controller

print 
print sys.path
print 


#Constants
#*************************************
NODE_NAME='PID'
#*************************************


class PID2(Controller):
  def __init__(self):
    self.load_PID_parameters()
    self.d_updated = 0 # Integral term
    rospy.Service('PID_controller/update_parameters', Empty, self.update_parameters)
  
    self.pid_x = pid_controller.PID(name="pid_x")
    self.pid_y = pid_controller.PID(name="pid_y")
    self.pid_z = pid_controller.PID(name="pid_z")

  # Resets PID
  def reset(self):
    self.d_updated = 0


  def get_d_updated(self):
    return self.d_updated


  def set_d_updated(self, d):
    self.d_updated = d


  def get_errors(self,current,target):
    e=[]
    for i in range (0,3):
      e.append(current[i]-target[i])

    return e

  # Returns the output of the controller
  def get_output(self,current,target):
    d_updated = self.get_d_updated()
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
    u=[0.0]*3
    
    time = rospy.get_rostime().to_sec()

    u[0] = self.pid_x.controller(x[0],x_target[0],time)
    u[1] = self.pid_y.controller(x[1],x_target[1],time)
    u[2] = self.pid_z.controller(x[2],x_target[2],time)

    print u
    return u


  def saturation(self,value,minimum,maximum):
    value=max(minimum,min(maximum,value))
    return value
	

  # Read parameters for PID
  def load_PID_parameters(self):		
    #Controller parameters	
    self.w = sml_setup.Get_Parameter(NODE_NAME,"PID_w",1.7)
    self.w_z  = sml_setup.Get_Parameter(NODE_NAME,"PID_w_z", 1.3)
    self.x_i = sml_setup.Get_Parameter(NODE_NAME,"PID_x_i",math.sqrt(2)/2)
    self.Kp = sml_setup.Get_Parameter(NODE_NAME,"PID_Kp",self.w*self.w)
    self.Kv = sml_setup.Get_Parameter(NODE_NAME,"PID_Kv",2*self.x_i*self.w)

    self.Kv_z = sml_setup.Get_Parameter(NODE_NAME,"PID_Kv_z", self.w_z*self.w_z)
    self.Kp_z = sml_setup.Get_Parameter(NODE_NAME,"PID_Kp_z", 2*self.x_i*self.w_z)

    self.I_lim = sml_setup.Get_Parameter(NODE_NAME,"PID_I_lim",0.5)
    self.K_i = sml_setup.Get_Parameter(NODE_NAME,"PID_K_i",7)

#EOF