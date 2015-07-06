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
from obstacle_avoidance import AvoidanceController
from PID_controller import Point, Instruction, PID




#Constants
#*************************************
NODE_NAME='Blender'
#*************************************

class Blender():

  def __init__(self):
    body_id = sml_setup.Get_Parameter(NODE_NAME,'body_id',8)
    body_array = sml_setup.Get_Parameter(NODE_NAME,'body_array',[1,2])
    self.PID = PID()
    self.avoidance = AvoidanceController(body_id,body_array)
    rospy.init_node(NODE_NAME)
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
    self.obstacle_avoidance = sml_setup.Get_Parameter(NODE_NAME,"obstacle_avoidance","False")
    
  def init_subscriptions(self, target_point,current_point, instr):
    #Subcribe to /trajectroy_gen/target to get target position, velocity and acceleration
    rospy.Subscriber('trajectory_gen/target',QuadPositionDerived,self.New_Point,target_point)
    #Subscribe to /derivator/pos_data to get position, velocity and acceleration
    rospy.Subscriber('security_guard/data_forward',QuadPositionDerived,self.New_Point,current_point)
    #Subscribe to /security_guard/controller to get permission to publish to rc/override
    rospy.Subscriber('security_guard/controller',Permission,self.Get_Permission,instr)

  def Get_Permission(self,data,instruction_obj):
    if instruction_obj.permission:
      if not data.permission:
        instruction_obj.permission=False

    if not instruction_obj.start:
      if data.permission:
        instruction_obj.start=True

  def Wait_For_Security_Guard(self,obj):
    rate=rospy.Rate(30)
    rospy.loginfo('['+NODE_NAME+']: Waiting for security guard ...')
    while not obj.start:
      if rospy.is_shutdown():
        return 
      rate.sleep()

  def Run_Blender(self):
    loop_rate=rospy.Rate(30)
    instr=Instruction()
    current_point=Point()
    target_point=Point()
    my_id = sml_setup.Get_Parameter(NODE_NAME,'body_id',1)
    bodies = sml_setup.Get_Parameter(NODE_NAME,'body_array',[1,2])
    #Publish to RC Override
    rc_override=rospy.Publisher('mavros/rc/override',OverrideRCIn,queue_size=10)

    self.init_subscriptions(target_point, current_point, instr)

    data_init=OverrideRCIn()
    command=[0,0,self.CONTROL_ARMING_MIN,0,0,0,0,0]
    data_init.channels=command

    #Wait until the security guard is online
    self.Wait_For_Security_Guard(instr)

    #integral term initialized to 0
    self.PID.set_d_updated(0)
    while not rospy.is_shutdown():
                

      if not target_point.first_point_received:
	self.Wait_For_First_Point(target_point,rc_override,data_init,loop_rate)
	#reinitialize d_updated
	self.PID.set_d_updated(0)
      d = self.PID.get_d_updated()
      x,x_vel,x_acc=self.PID.Get_Pos_Vel_Acc(current_point)  
      x_target,x_vel_target,x_acc_target=self.PID.Get_Pos_Vel_Acc(target_point) 
      u = self.blend(current_point, target_point,d)
      command_controlled = self.get_controloutput(u,x,x_target)
  
      #If OK from security guard, publish the messages via Mavros to the drone
      if instr.permission:
        data=OverrideRCIn()
        data.channels=command_controlled
        rc_override.publish(data)
  	
		
		

      loop_rate.sleep()
                
    
   

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

    norm_AUX=math.sqrt(math.pow(AUX_rot[0],2)+math.pow(AUX_rot[1],2)+math.pow(AUX_rot[2],2))

    #yaw control:
    diff=self.AngularDifference(x[3],x_target[3])
    w_yaw=-self.K_yaw*(math.radians(diff))

    #set values:
    throttle=(self.CONTROL_CANCEL_GRAVITY/9.8)*norm_AUX
    yaw=self.CONTROL_NEUTRAL - self.N_yaw*self.Saturation(w_yaw/self.w_inf,-1,1)
    pitch=self.CONTROL_NEUTRAL-self.Ktt*math.asin(AUX_rot[0]/norm_AUX)
    roll=self.CONTROL_NEUTRAL-self.Kphi*math.asin(AUX_rot[1]/norm_AUX)

    #if pitch<1400 or pitch>1600:
       #print(pitch)

    #if roll<1400 or roll>1600:
      #print(roll)

  #Implement some saturation
    throttle=self.Saturation(throttle,1000,2000)
    pitch=self.Saturation(pitch,1350,1650)
    roll=self.Saturation(roll,1350,1650)

    return [roll,pitch,throttle,yaw,0,0,0,0]


  def blend(self,current_point,target_point,d_updated):
    u = [0.,0.,0.]
    u_pid = self.PID.get_PID_output(current_point,target_point)
    u_obst = self.avoidance.get_potential_output()
    if self.obstacle_avoidance:
      alpha = self.avoidance.get_blending_constant()
    else:
      alpha = 0
    for i in range(0,2):
      u[i] = alpha * u_obst[i] + (1-alpha) * u_pid[i]
    u[2] = u_pid[2] 
    return u
 

  def New_Point(self,data,point_obj):
    if not point_obj.first_point_received:
      point_obj.first_point_received=True

    point_obj.update_point(data)

  def Saturation(self,value,minimum,maximum):

    value=max(minimum,min(maximum,value))

    return value


  def Wait_For_First_Point(self,target_obj,channel,data,rate):
    rospy.loginfo('['+NODE_NAME+']: Waiting for first point ...')
    while not target_obj.first_point_received:
      #publish low value on the throttle channel, so the drone does not disarm while waiting
      channel.publish(data)
      rate.sleep()

    rospy.loginfo('['+NODE_NAME+']: First point received')      

  def AngularDifference(self,current_angle,target_angle):
    ang_diff=current_angle-target_angle

    if math.fabs(ang_diff)>180:
      if ang_diff>0:
        ang_diff=ang_diff-360
      else:
        ang_diff=ang_diff+360

    return ang_diff
     

  

if __name__ == "__main__":
  bl = Blender()
  bl.Run_Blender()

#EOF
