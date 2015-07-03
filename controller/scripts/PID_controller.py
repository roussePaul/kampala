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


class PID:
	def __init__(self):
		self.load_PID_parameters()

	def AngularDifference(self,current_angle,target_angle):
		ang_diff=current_angle-target_angle

		if math.fabs(ang_diff)>180:
			if ang_diff>0:
				ang_diff=ang_diff-360
			else:
				ang_diff=ang_diff+360

		return ang_diff


	def Saturation(self,value,minimum,maximum):

		value=max(minimum,min(maximum,value))

		return value


	def Get_Errors(self,current,target):
		e=[]
		for i in range (0,3):
			e.append(current[i]-target[i])

		return e


	def PID(self,x,x_vel,x_acc,x_target,x_vel_target,x_acc_target,delta_t,current_d):
		u=[]
		AUX=[]
		AUX_rot=[]

		#Compute errors
		e=self.Get_Errors(x,x_target)
		e_dot=self.Get_Errors(x_vel,x_vel_target)

		new_d=current_d+delta_t*(self.K_i*((e[2]*self.Kv/2)+e_dot[2]))
		new_d=self.Saturation(new_d,-self.I_lim,self.I_lim)

		for i in range(0,2):
			u.append(x_acc_target[i]-self.Kv*e_dot[i]-self.Kp*e[i])
	        u.append(x_acc_target[2]-self.Kv_z*e_dot[2]-self.Kp_z*e[2])

		u[2]=u[2]-new_d

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

		return [roll,pitch,throttle,yaw,0,0,0,0],new_d


	def New_Point(self,data,point_obj):
		if not point_obj.first_point_received:
			point_obj.first_point_received=True

		point_obj.update_point(data)


	def Get_Permission(self,data,instruction_obj):
		if instruction_obj.permission:
			if not data.permission:
				instruction_obj.permission=False

		if not instruction_obj.start:
			if data.permission:
				instruction_obj.start=True


	def Wait_For_First_Point(self,target_obj,channel,data,rate):
		rospy.loginfo('['+NODE_NAME+']: Waiting for first point ...')
		while not target_obj.first_point_received:
			#publish low value on the throttle channel, so the drone does not disarm while waiting
			channel.publish(data)
			rate.sleep()
		self.load_PID_parameters()
		rospy.loginfo('['+NODE_NAME+']: First point received')


	def Get_Pos_Vel_Acc(self,obj):
		x=(obj.x,obj.y,obj.z,obj.yaw)
		x_vel=(obj.x_vel,obj.y_vel,obj.z_vel,obj.yaw_vel)
		x_acc=(obj.x_acc,obj.y_acc,obj.z_acc,obj.yaw_acc)

		return x,x_vel,x_acc


	def Wait_For_Security_Guard(self,obj):
		rate=rospy.Rate(30)
		rospy.loginfo('['+NODE_NAME+']: Waiting for security guard ...')
		while not obj.start:
			if rospy.is_shutdown():
				return 
			rate.sleep()

	def load_PID_parameters(self):
		self.CONTROL_MIN = sml_setup.Get_Parameter(NODE_NAME,"PID_CONTROL_MIN",1000)
		self.CONTROL_NEUTRAL = sml_setup.Get_Parameter(NODE_NAME,"PID_CONTROL_NEUTRAL",1500)
		self.CONTROL_MAX = sml_setup.Get_Parameter(NODE_NAME,"PID_CONTROL_MAX",2000)
		self.CONTROL_ARMING_MIN = sml_setup.Get_Parameter(NODE_NAME,"PID_CONTROL_ARMING_MIN",1025)
		self.CONTROL_CANCEL_GRAVITY = sml_setup.Get_Parameter(NODE_NAME,"PID_CONTROL_CANCEL_GRAVITY",1370)

			#Controller parameters
		self.Ktt = sml_setup.Get_Parameter(NODE_NAME,"PID_Ktt",1000)/(20*math.pi/180)
		self.Kphi = sml_setup.Get_Parameter(NODE_NAME,"PID_Kphi",1000)/(20*math.pi/180)

		self.w = sml_setup.Get_Parameter(NODE_NAME,"PID_w",1.1)
		self.w_z  = sml_setup.Get_Parameter(NODE_NAME,"PID_w_z ", 1.3)
		self.x_i = sml_setup.Get_Parameter(NODE_NAME,"PID_x_i",math.sqrt(2)/2)
		self.Kp = sml_setup.Get_Parameter(NODE_NAME,"PID_Kp",self.w*self.w)
		self.Kv = sml_setup.Get_Parameter(NODE_NAME,"PID_Kv",2*self.x_i*self.w)

		self.Kv_z = sml_setup.Get_Parameter(NODE_NAME,"PID_Kv_z", self.w_z*self.w_z)
		self.Kp_z = sml_setup.Get_Parameter(NODE_NAME,"PID_Kp_z", 2*self.x_i*self.w_z)

		self.N_yaw = sml_setup.Get_Parameter(NODE_NAME,"PID_N_yaw",500)
		self.K_yaw = sml_setup.Get_Parameter(NODE_NAME,"PID_K_yaw",2)
		self.w_inf = sml_setup.Get_Parameter(NODE_NAME,"PID_w_inf",5)

		self.I_lim = sml_setup.Get_Parameter(NODE_NAME,"PID_I_lim",0.5)
		self.K_i = sml_setup.Get_Parameter(NODE_NAME,"PID_K_i",7)

	def update_parameters(self,msg):
		utils.loginfo('PID parameters loaded')
		self.load_PID_parameters()
		return []

if __name__=='__main__':
	rospy.init_node('PID_controller')

	loop_rate=rospy.Rate(30)

	pid = PID()

	instr=Instruction()
	current_point=Point()
	target_point=Point()

	#Publish to RC Override
	rc_override=rospy.Publisher('mavros/rc/override',OverrideRCIn,queue_size=10)

	#Subcribe to /trajectroy_gen/target to get target position, velocity and acceleration
	rospy.Subscriber('trajectory_gen/target',QuadPositionDerived,pid.New_Point,target_point)
	#Subscribe to /derivator/pos_data to get position, velocity and acceleration
	rospy.Subscriber('security_guard/data_forward',QuadPositionDerived,pid.New_Point,current_point)
	#Subscribe to /security_guard/controller to get permission to publish to rc/override
	rospy.Subscriber('security_guard/controller',Permission,pid.Get_Permission,instr)

	rospy.Service('PID_controller/update_parameters', Empty, pid.update_parameters)

	data_init=OverrideRCIn()
	command=[0,0,pid.CONTROL_ARMING_MIN,0,0,0,0,0]
	data_init.channels=command


	#Wait until the security guard is online
	pid.Wait_For_Security_Guard(instr)

	#integral term initialized to 0
	d_updated=0

	while not rospy.is_shutdown():

		if not target_point.first_point_received:
			pid.Wait_For_First_Point(target_point,rc_override,data_init,loop_rate)
			#reinitialize d_updated
			d_updated=0

		#Extract the position, velocity and acceleration
		x,x_vel,x_acc=pid.Get_Pos_Vel_Acc(current_point)
		x_target,x_vel_target,x_acc_target=pid.Get_Pos_Vel_Acc(target_point)
		time_diff=current_point.time_diff

		#implement the PID controller
		command_controlled,d_updated=pid.PID(x,x_vel,x_acc,x_target,x_vel_target,x_acc_target,time_diff,d_updated)

		#If OK from security guard, publish the messages via Mavros to the drone
		if instr.permission:
			data=OverrideRCIn()
			data.channels=command_controlled
			rc_override.publish(data)


		loop_rate.sleep()


#EOF
