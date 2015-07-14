#!/usr/bin/env python

#PID controller for the IRIS+ in the SML Lab 
#Gets inputs from the Security Guard and the Trajectory Generator
#Publishes commands via Mavros' rc/override topic


import rospy
import sml_setup
import sys
import math
from controller_base import Controller
from PID_controller import PID
from point import *
from controller.msg import PlotData
from mavros.msg import OverrideRCIn
from mocap.msg import QuadPositionDerived
from controller.msg import Permission
from obstacle_avoidance import AvoidanceController
from std_srvs.srv import Empty
import utils
from std_msgs.msg import Float64

from numpy import linalg


#Constants
#*************************************
NODE_NAME='Blender'
#*************************************

class Blender():

  def __init__(self):
    self.PID = PID()
    self.avoidance = AvoidanceController()
    rospy.init_node(NODE_NAME)
    self.obstacle_avoidance = sml_setup.Get_Parameter(NODE_NAME,"obstacle_avoidance","False")
    rospy.Service('blender/update_parameters', Empty, self.update_parameters)
    self.instr = Instruction()
    self.load_parameters()
    
  # Gets target points and current points  
  def init_subscriptions(self, target_point,current_point):
    #Subcribe to /trajectroy_gen/target to get target position, velocity and acceleration
    rospy.Subscriber('trajectory_gen/target',QuadPositionDerived,self.new_point,target_point)
    #Subscribe to /derivator/pos_data to get position, velocity and acceleration
    rospy.Subscriber('security_guard/data_forward',QuadPositionDerived,self.new_point,current_point)
    #Subscribe to /security_guard/controller to get permission to publish to rc/override
    rospy.Subscriber('security_guard/controller',Permission,self.get_permission)

  def get_permission(self,data):
    if self.instr.permission:
      if not data.permission:
        self.instr.permission=False

    if not self.instr.start:
      if data.permission:
        self.instr.start=True

  def wait_for_security_guard(self,obj):
    rate=rospy.Rate(30)
    rospy.loginfo('['+NODE_NAME+']: Waiting for security guard ...')
    while not obj.start:
      if rospy.is_shutdown():
        return 
      rate.sleep()

  def run_blender(self):
    loop_rate=rospy.Rate(self.FREQUENCY)
    current_point=Point()
    target_point=Point()
    my_id = sml_setup.Get_Parameter(NODE_NAME,'body_id',1)
    bodies = sml_setup.Get_Parameter(NODE_NAME,'body_array',[1,2])
    # Publish to RC Override
    rc_override=rospy.Publisher('mavros/rc/override',OverrideRCIn,queue_size=10)
    d_pub = rospy.Publisher('theD',Float64,queue_size=10)

    self.init_subscriptions(target_point, current_point)

    data_init=OverrideRCIn()
    command=[self.CONTROL_NEUTRAL,self.CONTROL_NEUTRAL,self.CONTROL_ARMING_MIN,self.CONTROL_NEUTRAL,0,0,0,0]
    data_init.channels=command
    rc_override.publish(data_init)
    # Wait until the security guard is online
    self.wait_for_security_guard(self.instr)

    # Controller reset. For PID this means integral term initialized to 0.
    self.PID.reset()

    # Main loop
    while not rospy.is_shutdown():
      if not target_point.first_point_received:
        self.wait_for_first_point(target_point,rc_override,data_init,loop_rate)
        # Controller is reset. For the PID this means reinitialization of integral term.
        self.PID.reset()
      x,x_vel,x_acc=get_pos_vel_acc(current_point)  
      x_target,x_vel_target,x_acc_target=get_pos_vel_acc(target_point) 
      u = self.read_and_blend(current_point, target_point)
      command_controlled = self.get_controloutput(u,x,x_target)
  
      #If OK from security guard, publish the messages via Mavros to the drone
      if self.instr.permission:
        data=OverrideRCIn()
        data.channels=command_controlled
        rc_override.publish(data)
        d_pub.publish(self.PID.get_xi()[2])
      else:
        break

      loop_rate.sleep()
                

  # From the required acceleration u, he control outputs are calculated: roll, pitch,
  # throttle and yaw_rate
  def get_controloutput(self,u,x,x_target):
    AUX=[]
    AUX_rot=[]
    AUX.append(u[0])
    AUX.append(u[1])
    AUX.append(9.8+u[2])

    #take into consideration the yaw angle
    AUX_rot.append(math.cos(math.radians(-x[3]))*AUX[0]-math.sin(math.radians(-x[3]))*AUX[1])
    AUX_rot.append(math.sin(math.radians(-x[3]))*AUX[0]+math.cos(math.radians(-x[3]))*AUX[1])
    AUX_rot.append(AUX[2])

    norm_AUX=linalg.norm(AUX_rot)

    norm_AUX=math.sqrt(math.pow(AUX_rot[0],2)+math.pow(AUX_rot[1],2)+math.pow(AUX_rot[2],2))

    #yaw control:
    diff=self.angular_difference(x[3],x_target[3])
    w_yaw=-self.K_yaw*(math.radians(diff))

    #set values:
    throttle=(self.CONTROL_CANCEL_GRAVITY)*math.sqrt(norm_AUX/9.8)
    yaw_rate=self.CONTROL_NEUTRAL - self.N_yaw*self.saturation(w_yaw/self.w_inf,-1,1)
    pitch=self.CONTROL_NEUTRAL-self.Ktt*math.asin(AUX_rot[0]/norm_AUX)
    roll=self.CONTROL_NEUTRAL-self.Kphi*math.asin(AUX_rot[1]/norm_AUX)

    #if pitch<1400 or pitch>1600:
       #print(pitch)

    #if roll<1400 or roll>1600:
      #print(roll)

    # Implement some saturation
    throttle=self.saturation(throttle,1000,2000)
    pitch=self.saturation(pitch,1350,1650)
    roll=self.saturation(roll,1350,1650)

    return [roll,pitch,throttle,yaw_rate,0,0,0,0]

  # Read the outputs of the controller and collision avoidance, then "blends"
  # the outputs  
  def read_and_blend(self,current_point,target_point):
    u = [0.,0.,0.]
    u_cont = self.PID.get_output(current_point,target_point)
    u_obst = self.avoidance.get_potential_output()
    if self.obstacle_avoidance:
      alpha = self.avoidance.get_blending_constant()
    else:
      alpha = 0
    for i in range(0,2):
      u[i] = alpha * u_obst[i] + (1-alpha) * u_cont[i]
    u[2] = u_cont[2] 
    return u_cont
 

  def new_point(self,data,point_obj):
    if not point_obj.first_point_received:
      point_obj.first_point_received=True

    point_obj.update_point(data)

  def saturation(self,value,minimum,maximum):
    value=max(minimum,min(maximum,value))
    return value


  def wait_for_first_point(self,target_obj,channel,data,rate):
    rospy.loginfo('['+NODE_NAME+']: Waiting for first point ...')
    while not target_obj.first_point_received:
      #publish low value on the throttle channel, so the drone does not disarm while waiting
      channel.publish(data)
      rate.sleep()

    rospy.loginfo('['+NODE_NAME+']: First point received')      

  def angular_difference(self,current_angle,target_angle):
    ang_diff=current_angle-target_angle

    if math.fabs(ang_diff)>180:
      if ang_diff>0:
        ang_diff=ang_diff-360
      else:
        ang_diff=ang_diff+360

    return ang_diff
     
  def update_parameters(self,msg):
    utils.loginfo('PID parameters loaded')
    self.load_parameters()
    return []
  
  # Read parameters for Blender
  def load_parameters(self):		
    self.N_yaw = sml_setup.Get_Parameter(NODE_NAME,"N_yaw",500)
    self.K_yaw = sml_setup.Get_Parameter(NODE_NAME,"K_yaw",2)
    self.w_inf = sml_setup.Get_Parameter(NODE_NAME,"w_inf",5)
    self.Ktt = sml_setup.Get_Parameter(NODE_NAME,"Ktt",1000)/(20*math.pi/180)
    self.Kphi = sml_setup.Get_Parameter(NODE_NAME,"Kphi",1000)/(20*math.pi/180)
    self.CONTROL_MIN = sml_setup.Get_Parameter(NODE_NAME,"CONTROL_MIN",1000)
    self.CONTROL_NEUTRAL = sml_setup.Get_Parameter(NODE_NAME,"CONTROL_NEUTRAL",1500)
    self.CONTROL_MAX = sml_setup.Get_Parameter(NODE_NAME,"CONTROL_MAX",2000)
    self.CONTROL_ARMING_MIN = sml_setup.Get_Parameter(NODE_NAME,"CONTROL_ARMING_MIN",1025)
    self.CONTROL_CANCEL_GRAVITY = sml_setup.Get_Parameter(NODE_NAME,"CONTROL_CANCEL_GRAVITY",1400)	

    self.FREQUENCY = sml_setup.Get_Parameter(NODE_NAME,"CONTROLLER_FREQUENCY",30)

if __name__ == "__main__":
  bl = Blender()
  bl.run_blender()

#EOF
