#!/usr/bin/env python

# Blender script for the IRIS+ in the SML Lab 
# Gets inputs from the Security Guard and the Trajectory Generator
# Publishes commands via Mavros' rc/override topic
 

import rospy
import sml_setup
import sys
import math
from copy import deepcopy
import numpy
from controller_base import Controller
from PID_controller import PID
from load_transport_controller import LoadTransportController
from point import *
from controller.msg import PlotData
from controller.srv import SetChannel6
from mavros.msg import OverrideRCIn
from mocap.msg import QuadPositionDerived, QuadPositionDerivedExt
from controller.msg import Permission
from obstacle_avoidance import AvoidanceController
from std_srvs.srv import Empty
import utils
from std_msgs.msg import Float64
import numpy as np
from numpy import linalg


#Constants
#*************************************
NODE_NAME='Blender'
#*************************************

class Blender():

  def __init__(self):
    self.obstacle_avoidance = utils.Get_Parameter("obstacle_avoidance","False")
    self.avoidance = AvoidanceController()
    body_id = utils.Get_Parameter('body_id',8)
    self.load_id = utils.Get_Parameter('load_id',body_id)

    # Check what controller should be used
    self.controller_type = utils.Get_Parameter("controller_type","PID")
    if self.controller_type == "load_transport":
      self.controller = LoadTransportController()
    else:
      self.controller = PID()
    rospy.init_node(NODE_NAME)
    rospy.Service('blender/update_parameters', Empty, self.update_parameters)
    self.channel6 = 0
    self.instr = Instruction()
    self.load_parameters()
    rospy.Service('blender/set_channel6', SetChannel6, self.set_channel6)

  #Sets channel 6 to value
  def set_channel6(self,channel6):
    self.channel6 = channel6.value
    return True
    
    
  # Gets target points and current points  
  def init_subscriptions(self, target_point,current_point):
    #Subscribe to /derivator/pos_data to get position, velocity and acceleration
    rospy.Subscriber('security_guard/data_forward',QuadPositionDerived,self.new_point,current_point)
    #Subscribe to /security_guard/controller to get permission to publish to rc/override
    rospy.Subscriber('security_guard/controller',Permission,self.get_permission)

    
    if self.controller_type == "load_transport":
      #Subcribe to /trajectroy_gen/target to get target position, velocity and acceleration
      rospy.Subscriber('trajectory_gen/target_ext',QuadPositionDerivedExt,self.new_point,target_point)
    else:
      #Subcribe to /trajectroy_gen/target to get target position, velocity and acceleration
      rospy.Subscriber('trajectory_gen/target',QuadPositionDerived,self.new_point,target_point)

  # For load, if any
  def init_load_subscription(self, current_load_point):
    rospy.Subscriber("/body_data/id_"+str(self.load_id),QuadPositionDerived,self.new_point,current_load_point)

  def get_permission(self,data):
    if self.instr.permission:
      if not data.permission:
        self.instr.permission=False

    if not self.instr.start:
      if data.permission:
        self.instr.start=True

  def wait_for_security_guard(self,obj):
    rate=rospy.Rate(30)
    rospy.loginfo('Waiting for security guard ...')
    while not obj.start:
      if rospy.is_shutdown():
        return data_initdata_initdata_init
      rate.sleep()

  def run_blender(self):
    loop_rate=rospy.Rate(self.FREQUENCY)
    current_point=Point()
    current_load_point=Point()
    if self.controller_type == "load_transport":
      target_point=PointExt()
    else:
      target_point=Point()
    # Publish to RC Override
    rc_override=rospy.Publisher('mavros/rc/override',OverrideRCIn,queue_size=10)
    if type(self.controller) is PID:
      d_pub = rospy.Publisher('theD',Float64,queue_size=10)

    self.init_subscriptions(target_point, current_point)

    # For load
    self.init_load_subscription(current_load_point)

    data_init=OverrideRCIn()
    command=[self.CONTROL_NEUTRAL,self.CONTROL_NEUTRAL,self.CONTROL_ARMING_MIN,self.CONTROL_NEUTRAL,0,0,0,0]
    data_init.channels=command
    rc_override.publish(data_init)
    # Wait until the security guard is online
    self.wait_for_security_guard(self.instr)

    # Controller reset. For PID this means integral term initialized to 0.
    self.controller.reset()

    # Main loop
    while not rospy.is_shutdown():
      if not target_point.first_point_received:
        self.wait_for_first_point(target_point,rc_override,data_init,loop_rate)
        # Controller is reset. For the PID this means reinitialization of integral term.
        self.controller.reset()

      x,x_vel,x_acc=get_pos_vel_acc(current_point)  
      x_target,x_vel_target,x_acc_target=get_pos_vel_acc(target_point) 


      #current_load_point = deepcopy(current_point)
      #current_load_point.z = current_point.z - 5.0

      if type(self.controller) is PID:
        u_cont = self.controller.get_output(current_point,target_point)
        u_cont[2] = u_cont[2] + 9.8
      elif type(self.controller) is LoadTransportController:
        u_cont = self.controller.get_output(current_load_point,current_point,target_point)

      u = self.blend(u_cont, current_point, target_point)
      command_controlled = self.get_controloutput(u,x,x_target)
  
      #If OK from security guard, publish the messages via Mavros to the drone
      if self.instr.permission:
        data=OverrideRCIn()
        data.channels=command_controlled
        rc_override.publish(data)
        if type(self.controller) is PID:
          d_pub.publish(self.controller.get_d_updated()[1])
      else:
        break

      loop_rate.sleep()
                

  # From the required acceleration u, he control outputs are calculated: roll, pitch,
  # throttle and yaw_rate
  def get_controloutput(self,u,x,x_target):
    AUX=np.array([0.]*3)
    AUX_rot=np.array([0.]*3)
    AUX[0] = u[0]
    AUX[1] = u[1]
    AUX[2] = u[2]


    #take into consideration the yaw angle
    AUX_rot[0] = math.cos(math.radians(-x[3]))*AUX[0]-math.sin(math.radians(-x[3]))*AUX[1]
    AUX_rot[1] = math.sin(math.radians(-x[3]))*AUX[0]+math.cos(math.radians(-x[3]))*AUX[1]
    AUX_rot[2] = AUX[2]

    norm_AUX=linalg.norm(AUX_rot)

    #yaw control:
    diff=self.angular_difference(x[3],x_target[3])
    w_yaw=-self.K_yaw*(math.radians(diff))

    #set values:
    throttle=(self.CONTROL_CANCEL_GRAVITY)*math.sqrt(norm_AUX/9.8)
    yaw_rate=self.CONTROL_NEUTRAL - self.N_yaw*self.saturation(w_yaw/self.w_inf,-1,1)
    pitch=self.CONTROL_NEUTRAL-self.Ktt*math.asin(AUX_rot[0]/norm_AUX)
    roll=self.CONTROL_NEUTRAL-self.Kphi*math.asin(AUX_rot[1]/norm_AUX)


    # Implement some saturation
    throttle=self.saturation(throttle,1000,2000)
    pitch=self.saturation(pitch,1350,1700)
    roll=self.saturation(roll,1350,1700)
    return [roll,pitch,throttle,yaw_rate,0,self.channel6,0,0]


  # Read the outputs of the controller and collision avoidance, then "blends"
  # the outputs  
  def blend(self,u_cont,current_point,target_point):
    u = np.array([0.,0.,0.])
    u_obst = self.avoidance.get_potential_output()
    if self.obstacle_avoidance:
      alpha = self.avoidance.get_blending_constant()
<<<<<<< HEAD
=======
      utils.loginfo(alpha)
>>>>>>> cfbd46fb2883c9ccd5fec4764a888c83a1b082b0
    else:
      alpha = 0
    for i in range(0,2):
      u[i] = alpha * u_obst[i] + (1-alpha) * u_cont[i]
    u[2] = u_cont[2] 
    return u
 

  def new_point(self,data,point_obj):
    if not point_obj.first_point_received:
      point_obj.first_point_received=True

    point_obj.update_point(data)

  def saturation(self,value,minimum,maximum):
    value=max(minimum,min(maximum,value))
    return value


  def wait_for_first_point(self,target_obj,channel,data,rate):
    rospy.loginfo('Waiting for first point ...')
    while not target_obj.first_point_received:
      #publish low value on the throttle channel, so the drone does not disarm while waiting
      channel.publish(data)
      rate.sleep()

    rospy.loginfo('First point received')      

  def angular_difference(self,current_angle,target_angle):
    ang_diff=current_angle-target_angle

    if math.fabs(ang_diff)>180:
      if ang_diff>0:
        ang_diff=ang_diff-360
      else:
        ang_diff=ang_diff+360

    return ang_diff
     
  def update_parameters(self,msg):
    utils.loginfo('Blender parameters loaded')
    self.load_parameters()

    # If controller type is changed
    if self.controller_type == "load_transport" and type(self.controller) != LoadTransportController:
      self.controller = LoadTransportController()
    elif self.controller_type == "PID" and type(self.controller) != PID:
      self.controller = PID()

    return []
  
  # Read parameters for Blender
  def load_parameters(self):		
    self.N_yaw = utils.Get_Parameter("N_yaw",500)
    self.K_yaw = utils.Get_Parameter("K_yaw",2)
    self.w_inf = utils.Get_Parameter("w_inf",5)
    self.Ktt = utils.Get_Parameter("Ktt",1000)/(20*math.pi/180)
    self.Kphi = utils.Get_Parameter("Kphi",1000)/(20*math.pi/180)
    self.CONTROL_MIN = utils.Get_Parameter("CONTROL_MIN",1000)
    self.CONTROL_NEUTRAL = utils.Get_Parameter("CONTROL_NEUTRAL",1500)
    self.CONTROL_MAX = utils.Get_Parameter("CONTROL_MAX",2000)
    self.CONTROL_ARMING_MIN = utils.Get_Parameter("CONTROL_ARMING_MIN",1025)
    self.CONTROL_CANCEL_GRAVITY = utils.Get_Parameter("CONTROL_CANCEL_GRAVITY",1400)	

    self.FREQUENCY = utils.Get_Parameter("CONTROLLER_FREQUENCY",30)

    self.controller_type = utils.Get_Parameter("controller_type","PID")

if __name__ == "__main__":
  bl = Blender()
  bl.run_blender()

#EOF
