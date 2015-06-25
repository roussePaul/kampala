#!/usr/bin/env python

import rospy
import mocap
import sys
import ast
import sml_setup

from mocap.msg import QuadPosition
from mocap.msg import QuadPositionDerived

#************Constants********************
NODE_NAME='MOCAP'
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


def Insert_Current_Data(current_data):
	result=QuadPositionDerived()
	result.found_body=current_data.found_body
	result.x=current_data.x
	result.y=current_data.y
	result.z=current_data.z
	result.pitch=current_data.pitch
	result.roll=current_data.roll
	result.yaw=current_data.yaw
	return(result)


def Compute_Derivative(current,past,time):
	result=(current-past)/time
	return(result)


def Get_Derived_Data(current_data,past_data,time):
	result=Insert_Current_Data(current_data)
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
	return(result)


def start_publishing():
	rate=rospy.Rate(30)
	timer=Time()
	#Get parameters (all the body ID's that are requested)
	body_array=sml_setup.Get_Parameter(NODE_NAME,'body_array',[8,12])
	if type(body_array) is str:
		body_array=ast.literal_eval(body_array)

	#Get topic names for each requested body
	body_topic_array=Get_Topic_Names(body_array)

	#Establish one publisher topic for each requested body
	topics_publisher=Get_Publishers(body_topic_array)

	#Initialize empty past data list
	mocap_past_data=[]
	empty_data=QuadPositionDerived()
	for i in range(0,len(body_array)):
		mocap_past_data.append(empty_data)

	while not rospy.is_shutdown():

		delta_time=timer.get_time_diff()

		for i in range(0,len(body_array)):
			mocap_data=Get_Body_Data(body_array[i])
			mocap_data_derived=Get_Derived_Data(mocap_data,mocap_past_data[i],delta_time)

			#update past mocap data
			mocap_past_data[i]=mocap_data_derived

			#Publish data on topic
			topics_publisher[i].publish(mocap_data_derived)

		rate.sleep()


if __name__=="__main__":
	rospy.init_node("ros_mocap_unfiltered")
	Qs=mocap.Mocap(info=0)
	bodies=Qs.get_updated_bodies()
	if(bodies=="off"):
		rospy.logerr("No connection to the Qualisys Motion Capture System")
		sys.exit()
	else:
		start_publishing()

#EOF
