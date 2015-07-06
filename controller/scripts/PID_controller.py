#!/usr/bin/env python

#PID controller for the IRIS+ in the SML Lab 
#Gets inputs from the Security Guard and the Trajectory Generator
#Publishes commands via Mavros' rc/override topic


import rospy
import sml_setup
import sys
import math
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


class Point():
	def __init__(self):
		self.found_body=True
		self.x=0
		self.y=0
		self.z=0
		self.yaw=0
		self.pitch=0
		self.roll=0
		self.x_vel=0
		self.y_vel=0
		self.z_vel=0
		self.yaw_vel=0
		self.pitch_vel=0
		self.roll_vel=0
		self.x_acc=0
		self.y_acc=0
		self.z_acc=0
		self.yaw_acc=0
		self.pitch_acc=0
		self.roll_acc=0
		self.time_diff=0
		self.first_point_received=False

	def update_point(self,new_data):
		self.found_body=new_data.found_body
		self.x=new_data.x
		self.y=new_data.y
		self.z=new_data.z
		self.yaw=new_data.yaw
		self.pitch=new_data.pitch
		self.roll=new_data.roll
		self.x_vel=new_data.x_vel
		self.y_vel=new_data.y_vel
		self.z_vel=new_data.z_vel
		self.yaw_vel=new_data.yaw_vel
		self.pitch_vel=new_data.pitch_vel
		self.roll_vel=new_data.roll_vel
		self.x_acc=new_data.x_acc
		self.y_acc=new_data.y_acc
		self.z_acc=new_data.z_acc
		self.yaw_acc=new_data.yaw_acc
		self.pitch_acc=new_data.pitch_acc
		self.roll_acc=new_data.roll_acc
		self.time_diff=new_data.time_diff


class Instruction():
	def __init__(self):
		self.start=False
		self.permission=True


class PID():
  def __init__(self):
    self.load_PID_parameters()
    self.d_updated = 0

    rospy.Service('PID_controller/update_parameters', Empty, pid.update_parameters)
  def get_d_updated(self):
    return self.d_updated

  def set_d_updated(self, d):
    self.d_updated = d


  def Get_Errors(self,current,target):
    e=[]
    for i in range (0,3):
      e.append(current[i]-target[i])

    return e


  def get_PID_output(self,current, target):
    d_updated = self.get_d_updated()
    x,x_vel,x_acc=self.Get_Pos_Vel_Acc(current)  #current state
    x_target,x_vel_target,x_acc_target=self.Get_Pos_Vel_Acc(target) #target state
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
    e=self.Get_Errors(x,x_target)
    e_dot=self.Get_Errors(x_vel,x_vel_target)

    new_d=current_d+delta_t*(self.K_i*((e[2]*self.Kv/2)+e_dot[2]))
    new_d=self.Saturation(new_d,-self.I_lim,self.I_lim)

    for i in range(0,2):
      u.append(x_acc_target[i]-self.Kv*e_dot[i]-self.Kp*e[i])
    u.append(x_acc_target[2]-self.Kv_z*e_dot[2]-self.Kp_z*e[2])

    u[2]=u[2]-new_d
    self.set_d_updated(new_d)    

    return u

  def Saturation(self,value,minimum,maximum):
    value=max(minimum,min(maximum,value))
    return value



  def Get_Pos_Vel_Acc(self,obj):
    x=(obj.x,obj.y,obj.z,obj.yaw)
    x_vel=(obj.x_vel,obj.y_vel,obj.z_vel,obj.yaw_vel)
    x_acc=(obj.x_acc,obj.y_acc,obj.z_acc,obj.yaw_acc)
    return x,x_vel,x_acc


	
  def load_PID_parameters(self):		
    #Controller parameters	
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


#EOF
