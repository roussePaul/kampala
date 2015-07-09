#!/usr/bin/env python

#PID controller for the IRIS+ in the SML Lab 
#Gets inputs from the Security Guard and the Trajectory Generator
#Publishes commands via Mavros' rc/override topic


import rospy
import sml_setup
import sys
import math
from points import *
from controller.msg import PlotData
from mavros.msg import OverrideRCIn
from mocap.msg import QuadPositionDerived
from controller.msg import Permission
from std_srvs.srv import Empty
import analysis
import utils


#Constants
#*************************************
NODE_NAME='PID'
#*************************************


class PID():
  def __init__(self):
    self.load_PID_parameters()
    self.d_updated = 0
    rospy.Service('PID_controller/update_parameters', Empty, self.update_parameters)
  
  def get_d_updated(self):
    return self.d_updated

  def set_d_updated(self, d):
    self.d_updated = d


  def get_errors(self,current,target):
    e=[]
    for i in range (0,3):
      e.append(current[i]-target[i])

    return e


  def get_PID_output(self,current, target):
    d_updated = self.get_d_updated()
    x,x_vel,x_acc=get_pos_vel_acc(current)  #current state
    x_target,x_vel_target,x_acc_target=get_pos_vel_acc(target) #target state
    time_diff=current.time_diff 
    command_controlled=self.calculate_PID_output(x,x_vel,x_acc,x_target,x_vel_target,x_acc_target,time_diff,d_updated)
    return command_controlled


  def update_parameters(self,msg):
    utils.loginfo('PID parameters loaded')
    self.load_PID_parameters()
    return []


  def calculate_PID_output(self,x,x_vel,x_acc,x_target,x_vel_target,x_acc_target,delta_t,current_d):
    u=[]
    #Compute errors
    e=self.get_errors(x,x_target)
    e_dot=self.get_errors(x_vel,x_vel_target)

    new_d=current_d+delta_t*(self.K_i*((e[2]*self.Kv/2)+e_dot[2]))
    new_d=self.saturation(new_d,-self.I_lim,self.I_lim)

    for i in range(0,2):
      u.append(x_acc_target[i]-self.Kv*e_dot[i]-self.Kp*e[i])
    u.append(x_acc_target[2]-self.Kv_z*e_dot[2]-self.Kp_z*e[2])

    u[2]=u[2]-new_d
    self.set_d_updated(new_d)    

    return u


  def saturation(self,value,minimum,maximum):
    value=max(minimum,min(maximum,value))
    return value
	

  def load_PID_parameters(self):		
    #Controller parameters	
    self.N_yaw = sml_setup.Get_Parameter(NODE_NAME,"PID_N_yaw",500)
    self.K_yaw = sml_setup.Get_Parameter(NODE_NAME,"PID_K_yaw",2)
    self.w_inf = sml_setup.Get_Parameter(NODE_NAME,"PID_w_inf",5)
    self.Ktt = sml_setup.Get_Parameter(NODE_NAME,"PID_Ktt",1000)/(20*math.pi/180)
    self.Kphi = sml_setup.Get_Parameter(NODE_NAME,"PID_Kphi",1000)/(20*math.pi/180)
    self.CONTROL_MIN = sml_setup.Get_Parameter(NODE_NAME,"PID_CONTROL_MIN",1000)
    self.CONTROL_NEUTRAL = sml_setup.Get_Parameter(NODE_NAME,"PID_CONTROL_NEUTRAL",1500)
    self.CONTROL_MAX = sml_setup.Get_Parameter(NODE_NAME,"PID_CONTROL_MAX",2000)
    self.CONTROL_ARMING_MIN = sml_setup.Get_Parameter(NODE_NAME,"PID_CONTROL_ARMING_MIN",1025)
    self.CONTROL_CANCEL_GRAVITY = sml_setup.Get_Parameter(NODE_NAME,"PID_CONTROL_CANCEL_GRAVITY",1400)
    self.CONTROL_MIN = sml_setup.Get_Parameter(NODE_NAME,"PID_CONTROL_MIN",1000)
    self.CONTROL_NEUTRAL = sml_setup.Get_Parameter(NODE_NAME,"PID_CONTROL_NEUTRAL",1500)
    self.CONTROL_MAX = sml_setup.Get_Parameter(NODE_NAME,"PID_CONTROL_MAX",2000)
    self.CONTROL_ARMING_MIN = sml_setup.Get_Parameter(NODE_NAME,"PID_CONTROL_ARMING_MIN",1025)
    self.CONTROL_CANCEL_GRAVITY = sml_setup.Get_Parameter(NODE_NAME,"PID_CONTROL_CANCEL_GRAVITY",1370)
    self.w = sml_setup.Get_Parameter(NODE_NAME,"PID_w",1.7)
    self.w_z  = sml_setup.Get_Parameter(NODE_NAME,"PID_w_z", 1.3)
    self.x_i = sml_setup.Get_Parameter(NODE_NAME,"PID_x_i",math.sqrt(2)/2)
    self.Kp = sml_setup.Get_Parameter(NODE_NAME,"PID_Kp",self.w*self.w)
    self.Kv = sml_setup.Get_Parameter(NODE_NAME,"PID_Kv",2*self.x_i*self.w)

    self.Kv_z = sml_setup.Get_Parameter(NODE_NAME,"PID_Kv_z", self.w_z*self.w_z)
    self.Kp_z = sml_setup.Get_Parameter(NODE_NAME,"PID_Kp_z", 2*self.x_i*self.w_z)

    self.I_lim = sml_setup.Get_Parameter(NODE_NAME,"PID_I_lim",0.5)
    self.K_i = sml_setup.Get_Parameter(NODE_NAME,"PID_K_i",7)
    self.FREQUENCY = sml_setup.Get_Parameter(NODE_NAME,"CONTROLLER_FREQUENCY",30)


#EOF
