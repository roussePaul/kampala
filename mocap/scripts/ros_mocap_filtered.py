#!/usr/bin/env python

import rospy
import mocap
import sys
import ast
import sml_setup
import tf
import numpy as np
import std_msgs.msg

from mocap.msg import QuadPosition
from mocap.msg import QuadPositionDerived

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose

from scipy import signal

#************Constants********************
NODE_NAME='MOCAP'

# Filter configuration
SIZE_BUFFER=100
T_pos=0.05
T_vel=0.05
T_acc=0.05
#*****************************************


class Time():
	def __init__(self):
		self.current_time=rospy.Time.now()
		self.past_time=self.current_time
		self.time_diff=0

	def get_time_diff(self):
		self.past_time=self.current_time
		self.current_time=rospy.Time.now()
		d_time=self.current_time-self.past_time
		self.time_diff=d_time.secs+(d_time.nsecs/1E9)

		return self.time_diff


def Get_Body_Data(body_id):
	bodies=Qs.get_updated_bodies()
	body_length=len(bodies)
	body_indice=-1

	if isinstance(bodies,list):
		for i in range(0,body_length):
			if(bodies[i]['id']==body_id):
				body_indice=i

	data=QuadPosition()

	if(body_indice==-1):
		data.found_body=False
		return(data)
	
	
	data.found_body=True
	data.x=bodies[body_indice]["x"]
	data.y=bodies[body_indice]["y"]
	data.z=bodies[body_indice]["z"]
	data.pitch=bodies[body_indice]["pitch"]
	data.roll=bodies[body_indice]["roll"]
	data.yaw=bodies[body_indice]["yaw"]

	return(data)


def Get_Topic_Names(bodies):
	a=len(bodies)
	result=[]
	for i in range(0,a):
		result.append('body_data/id_'+str(bodies[i]))

	return(result)


def Get_Publishers(topic_array):
	a=len(topic_array)
	result=[]
	for i in range(0,a):
		result.append(rospy.Publisher(topic_array[i],QuadPositionDerived,queue_size=10))

	return(result)


def Insert_Current_Data(current_data,time):
	result=QuadPositionDerived()
	result.found_body=current_data.found_body
	result.x=current_data.x
	result.y=current_data.y
	result.z=current_data.z
	result.pitch=current_data.pitch
	result.roll=current_data.roll
	result.yaw=current_data.yaw
	result.time_diff=time
	return(result)


def Compute_Derivative(current,past,time):
	result=(current-past)/time
	return(result)


def Get_Derived_Data(input_signal,current_data,past_data,time):
	result=Insert_Current_Data(current_data,time)
	result.x_vel=Compute_Derivative(current_data.x,past_data.x,time)
	result.y_vel=Compute_Derivative(current_data.y,past_data.y,time)
	result.z_vel=Compute_Derivative(current_data.z,past_data.z,time)
	result.pitch_vel=Compute_Derivative(current_data.pitch,past_data.pitch,time)
	result.roll_vel=Compute_Derivative(current_data.roll,past_data.roll,time)
	result.yaw_vel=Compute_Derivative(current_data.yaw,past_data.yaw,time)
	result.x_acc=Compute_Derivative(result.x_vel,past_data.x_vel,time)
	result.y_acc=Compute_Derivative(result.y_vel,past_data.y_vel,time)
	result.z_acc=Compute_Derivative(result.z_vel,past_data.z_vel,time)
	result.pitch_acc=Compute_Derivative(result.pitch_vel,past_data.pitch_vel,time)
	result.roll_acc=Compute_Derivative(result.roll_vel,past_data.roll_vel,time)
	result.yaw_acc=Compute_Derivative(result.yaw_vel,past_data.yaw_vel,time)

	if current_data.found_body:
		Add_Data(input_signal,result)
	
	result = Apply_Filter(input_signal)
	result.found_body=current_data.found_body

	return(result)

def Add_Data(input_signal,current_data):
	if len(input_signal)>SIZE_BUFFER:
		input_signal.pop(0)

	input_signal.append(current_data)

def Apply_Filter_sig(sig,deltaT,T):
	#N=10
	#return np.mean(sig[len(sig)-N:len(sig)])
	if len(sig):
		i = 0
		t=0.0
		m=0.0
		while t<T and len(sig)>=i:
			t+=deltaT[-i]
			m+=sig[-i]
			i+=1
		return m/i
	else:
		return 0

def Get_Signal_From_Dict(data,key):
	return [d.__getattribute__(key) for d in data]

def Apply_Filter(input_signal):
	result = QuadPositionDerived()
	keys_pos = ['x','y','z','pitch','roll','yaw']
	keys_vel = ['x_vel','y_vel','z_vel','pitch_vel','roll_vel','yaw_vel']
	keys_acc = ['x_acc','y_acc','z_acc','pitch_acc','roll_acc','yaw_acc']
	deltaT = Get_Signal_From_Dict(input_signal,'time_diff')
	for k in keys_pos:
		s = Get_Signal_From_Dict(input_signal,k)
		r = Apply_Filter_sig(s,deltaT,T_pos)
		result.__setattr__(k,r)
	
	for k in keys_vel:
		s = Get_Signal_From_Dict(input_signal,k)
		r = Apply_Filter_sig(s,deltaT,T_vel)
		result.__setattr__(k,r)
	
	for k in keys_acc:
		s = Get_Signal_From_Dict(input_signal,k)
		r = Apply_Filter_sig(s,deltaT,T_vel)
		result.__setattr__(k,r)
	
	return result


class GetData:
	def __init__(self):
		rospy.Timer(rospy.Duration(0.01), self.mocap_get_data_callback)

		rate=rospy.Rate(30)
		self.timer=Time()
		#Get parameters (all the body ID's that are requested)
		self.body_array=sml_setup.Get_Parameter(NODE_NAME,'body_array',[8,12])
		if type(self.body_array) is str:
			self.body_array=ast.literal_eval(self.body_array)

		#Get topic names for each requested body
		body_topic_array=Get_Topic_Names(self.body_array)

		#Establish one publisher topic for each requested body
		topics_publisher=Get_Publishers(body_topic_array)

		#Initialize empty past data list
		self.mocap_past_data=[]
		empty_data=QuadPositionDerived()
		for i in range(0,len(self.body_array)):
			self.mocap_past_data.append(empty_data)

		#Initialize buffer
		self.input_signal=[]
		for i in range(0,len(self.body_array)):
			self.input_signal.append([])

		while not rospy.is_shutdown():
			for i in range(0,len(self.body_array)):
				topics_publisher[i].publish(self.mocap_past_data[i])
			rate.sleep()

	def mocap_get_data_callback(self,event):
		delta_time=self.timer.get_time_diff()
		for i in range(0,len(self.body_array)):
			mocap_data=Get_Body_Data(self.body_array[i])
			mocap_data_derived=Get_Derived_Data(self.input_signal[i],mocap_data,self.mocap_past_data[i],delta_time)

			#update past mocap data
			self.mocap_past_data[i]=mocap_data_derived

if __name__=="__main__":
	rospy.init_node("ros_mocap")
	Qs=mocap.Mocap(info=0)
	bodies=Qs.get_updated_bodies()
	if(bodies=="off"):
		rospy.logerr("No connection to the Qualisys Motion Capture System")
		sys.exit()
	else:
		GetData()

#EOF
