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


#Constants
#*************************************
NODE_NAME='PID'

CONTROL_MIN=1000
CONTROL_NEUTRAL=1500
CONTROL_MAX=2000
CONTROL_ARMING_MIN=1025
CONTROL_CANCEL_GRAVITY=1500 #Might have to be changed for different quads!

#Controller parameters
Ktt=1000/(20*math.pi/180)
Kphi=1000/(20*math.pi/180)

w=1.7
w_z = 1.8
x_i=math.sqrt(2)/2
Kp=w*w
Kv=2*x_i*w

Kv_z= w_z*w_z
Kp_z= 2*x_i*w_z

N_yaw=500
K_yaw=2
w_inf=5

I_lim=0.5
K_i=7
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



def AngularDifference(current_angle,target_angle):
	ang_diff=current_angle-target_angle

	if math.fabs(ang_diff)>180:
		if ang_diff>0:
			ang_diff=ang_diff-360
		else:
			ang_diff=ang_diff+360

	return ang_diff


def Saturation(value,minimum,maximum):

	value=max(minimum,min(maximum,value))

	return value


def Get_Errors(current,target):
	e=[]
	for i in range (0,3):
		e.append(current[i]-target[i])

	return e


def PID(x,x_vel,x_acc,x_target,x_vel_target,x_acc_target,delta_t,current_d):
	u=[]
	AUX=[]
	AUX_rot=[]

	#Compute errors
	e=Get_Errors(x,x_target)
	e_dot=Get_Errors(x_vel,x_vel_target)

	new_d=current_d+delta_t*(K_i*((e[2]*Kv/2)+e_dot[2]))
	new_d=Saturation(new_d,-I_lim,I_lim)

	for i in range(0,2):
		u.append(x_acc_target[i]-Kv*e_dot[i]-Kp*e[i])
        u.append(x_acc_target[2]-Kv_z*e_dot[2]-Kp_z*e[2])

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
	diff=AngularDifference(x[3],x_target[3])
	w_yaw=-K_yaw*(math.radians(diff))

	#set values:
	throttle=(CONTROL_CANCEL_GRAVITY/9.8)*norm_AUX
	yaw=CONTROL_NEUTRAL - N_yaw*Saturation(w_yaw/w_inf,-1,1)
	pitch=CONTROL_NEUTRAL-Ktt*math.asin(AUX_rot[0]/norm_AUX)
	roll=CONTROL_NEUTRAL-Kphi*math.asin(AUX_rot[1]/norm_AUX)

	#if pitch<1400 or pitch>1600:
                #print(pitch)

	#if roll<1400 or roll>1600:
                #print(roll)

	#Implement some saturation
	throttle=Saturation(throttle,1000,2000)
	pitch=Saturation(pitch,1350,1650)
	roll=Saturation(roll,1350,1650)

	return [roll,pitch,throttle,yaw,0,0,0,0],new_d


def New_Point(data,point_obj):
	if not point_obj.first_point_received:
		point_obj.first_point_received=True

	point_obj.update_point(data)


def Get_Permission(data,instruction_obj):
	if instruction_obj.permission:
		if not data.permission:
			instruction_obj.permission=False

	if not instruction_obj.start:
		if data.permission:
			instruction_obj.start=True


def Wait_For_First_Point(target_obj,channel,data,rate):
	rospy.loginfo('['+NODE_NAME+']: Waiting for first point ...')
	while not target_obj.first_point_received:
		#publish low value on the throttle channel, so the drone does not disarm while waiting
		channel.publish(data)
		rate.sleep()

	rospy.loginfo('['+NODE_NAME+']: First point received')


def Get_Pos_Vel_Acc(obj):
	x=(obj.x,obj.y,obj.z,obj.yaw)
	x_vel=(obj.x_vel,obj.y_vel,obj.z_vel,obj.yaw_vel)
	x_acc=(obj.x_acc,obj.y_acc,obj.z_acc,obj.yaw_acc)

	return x,x_vel,x_acc


def Wait_For_Security_Guard(obj):
	rate=rospy.Rate(30)
	rospy.loginfo('['+NODE_NAME+']: Waiting for security guard ...')
	while not obj.start:
		rate.sleep()



if __name__=='__main__':
	rospy.init_node('PID_controller')

	loop_rate=rospy.Rate(30)

	instr=Instruction()
	current_point=Point()
	target_point=Point()

	#Publish to RC Override
	rc_override=rospy.Publisher('mavros/rc/override',OverrideRCIn,queue_size=10)

	#Subcribe to /trajectroy_gen/target to get target position, velocity and acceleration
	rospy.Subscriber('trajectory_gen/target',QuadPositionDerived,New_Point,target_point)
	#Subscribe to /derivator/pos_data to get position, velocity and acceleration
	rospy.Subscriber('security_guard/data_forward',QuadPositionDerived,New_Point,current_point)
	#Subscribe to /security_guard/controller to get permission to publish to rc/override
	rospy.Subscriber('security_guard/controller',Permission,Get_Permission,instr)

	data_init=OverrideRCIn()
	command=[0,0,CONTROL_ARMING_MIN,0,0,0,0,0]
	data_init.channels=command

	#Wait until the security guard is online
	Wait_For_Security_Guard(instr)

	#integral term initialized to 0
	d_updated=0

	while not rospy.is_shutdown():

		if not target_point.first_point_received:
			Wait_For_First_Point(target_point,rc_override,data_init,loop_rate)
			#reinitialize d_updated
			d_updated=0

		#Extract the position, velocity and acceleration
		x,x_vel,x_acc=Get_Pos_Vel_Acc(current_point)
		x_target,x_vel_target,x_acc_target=Get_Pos_Vel_Acc(target_point)
		time_diff=current_point.time_diff

		#implement the PID controller
		command_controlled,d_updated=PID(x,x_vel,x_acc,x_target,x_vel_target,x_acc_target,time_diff,d_updated)

		#If OK from security guard, publish the messages via Mavros to the drone
		if instr.permission:
			data=OverrideRCIn()
			data.channels=command_controlled
			rc_override.publish(data)


		loop_rate.sleep()


#EOF
