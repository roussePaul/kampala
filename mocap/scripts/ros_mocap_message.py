#!/usr/bin/env python

import rospy
import sys
import ast
import sml_setup

from mocap.msg import QuadPosition
from mocap.msg import QuadPositionDerived

#************Constants********************
NODE_NAME='MOCAP'
#*****************************************

from mocap.msg import QuadPosition
from mocap.srv import Bodies
from mocap.srv import BodyData
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

from rotmat import Vector3, Matrix3
from math import radians, degrees

from scipy import signal

import analysis
import utils

def quat_to_dcm(q1, q2, q3, q4):
	'''convert quaternion to DCM'''
	q3q3 = q3 * q3
	q3q4 = q3 * q4
	q2q2 = q2 * q2
	q2q3 = q2 * q3
	q2q4 = q2 * q4
	q1q2 = q1 * q2
	q1q3 = q1 * q3
	q1q4 = q1 * q4
	q4q4 = q4 * q4

	m = Matrix3()
	m.a.x = 1.0-2.0*(q3q3 + q4q4)
	m.a.y =   2.0*(q2q3 - q1q4)
	m.a.z =   2.0*(q2q4 + q1q3)
	m.b.x =   2.0*(q2q3 + q1q4)
	m.b.y = 1.0-2.0*(q2q2 + q4q4)
	m.b.z =   2.0*(q3q4 - q1q2)
	m.c.x =   2.0*(q2q4 - q1q3)
	m.c.y =   2.0*(q3q4 + q1q2)
	m.c.z = 1.0-2.0*(q2q2 + q3q3)
	return m

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

class Mocap:
	def __init__(self):
		utils.loginfo('Mocap starting')
		self.bodies = []
		self.start_subscribes()
		self.start_publishing()

	def get_bodies(self,arg):
		num_bodies = len(self.bodies.name)
		return {'list':range(num_bodies)}


	def get_data(self,body):
		data = QuadPosition()

		body_id = body

		data.found_body = False
		
		if self.bodies==[]:
			utils.loginfo("Gazebo not started")
		else:
			if hasattr(self.bodies,'name'):
				if body_id<len(self.bodies.name):
					data.found_body=True
					data.x=self.bodies.pose[body_id].position.x
					data.y=self.bodies.pose[body_id].position.y
					data.z=self.bodies.pose[body_id].position.z
					x = self.bodies.pose[body_id].orientation.x
					y = self.bodies.pose[body_id].orientation.y
					z = self.bodies.pose[body_id].orientation.z
					w = self.bodies.pose[body_id].orientation.w

					dcm = quat_to_dcm(w,x,y,z)
					(roll,pitch,yaw) = dcm.to_euler()

					data.pitch=degrees(pitch)
					data.roll=degrees(roll)
					data.yaw=degrees(yaw)
					
					print "Sending data for the body with id: "+str(body_id)
				else:
					print 'Body not found'
			
		return(data)

	def update_positions(self,msg):
		utils.loginfo('receive data from Gazebo')
		self.bodies = msg


	def start_subscribes(self):
		utils.loginfo('Suscrib')
		rospy.Subscriber("/gazebo/model_states",ModelStates,self.update_positions)

	def Get_Topic_Names(self,bodies):
		a=len(bodies)
		result=[]
		for i in range(0,a):
			result.append('body_data/id_'+str(self.body_array[i]))

		return(result)


	def Get_Publishers(self,topic_array):
		a=len(topic_array)
		result=[]
		for i in range(0,a):
			result.append(rospy.Publisher(topic_array[i],QuadPositionDerived,queue_size=10))

		return(result)


	def Insert_Current_Data(self,current_data):
		result=QuadPositionDerived()
		result.found_body=current_data.found_body
		result.x=current_data.x
		result.y=current_data.y
		result.z=current_data.z
		result.pitch=current_data.pitch
		result.roll=current_data.roll
		result.yaw=current_data.yaw
		return(result)


	def Compute_Derivative(self,current,past,time):
		if time==0:
			return 0
		result=(current-past)/time
		return(result)


	def Get_Derived_Data(self,current_data,past_data,time):
		result=self.Insert_Current_Data(current_data)
		result.x_vel=self.Compute_Derivative(current_data.x,past_data.x,time)
		result.y_vel=self.Compute_Derivative(current_data.y,past_data.y,time)
		result.z_vel=self.Compute_Derivative(current_data.z,past_data.z,time)
		result.pitch_vel=self.Compute_Derivative(current_data.pitch,past_data.pitch,time)
		result.roll_vel=self.Compute_Derivative(current_data.roll,past_data.roll,time)
		result.yaw_vel=self.Compute_Derivative(current_data.yaw,past_data.yaw,time)
		result.x_acc=self.Compute_Derivative(result.x_vel,past_data.x_vel,time)
		result.y_acc=self.Compute_Derivative(result.y_vel,past_data.y_vel,time)
		result.z_acc=self.Compute_Derivative(result.z_vel,past_data.z_vel,time)
		result.pitch_acc=self.Compute_Derivative(result.pitch_vel,past_data.pitch_vel,time)
		result.roll_acc=self.Compute_Derivative(result.roll_vel,past_data.roll_vel,time)
		result.yaw_acc=self.Compute_Derivative(result.yaw_vel,past_data.yaw_vel,time)
		return(result)


	def start_publishing(self):
		
		utils.loginfo('start publishing')
		
		rate=rospy.Rate(30)
		timer=Time()
		#Get parameters (all the body ID's that are requested)
		self.body_names=sml_setup.Get_Parameter(NODE_NAME,'body_names',['iris1','iris2'])
		self.body_array=sml_setup.Get_Parameter(NODE_NAME,'body_array',[1,2])
		if type(self.body_array) is str:
			self.body_array=ast.literal_eval(self.body_array)

		#Get topic names for each requested body
		body_topic_array=self.Get_Topic_Names(self.body_array)

		#Establish one publisher topic for each requested body
		topics_publisher=self.Get_Publishers(body_topic_array)

		#Initialize empty past data list
		mocap_past_data=[]
		empty_data=QuadPositionDerived()
		for i in range(0,len(self.body_array)):
			mocap_past_data.append(empty_data)

		while not rospy.is_shutdown():

			delta_time=timer.get_time_diff()

			if self.bodies:
				for i in range(0,len(self.body_array)):
					utils.loginfo('body ' + str(i))
					if self.body_names[i] in self.bodies.name:
						indice = self.bodies.name.index(self.body_names[i])

						mocap_data=self.get_data(indice)
						mocap_data_derived=self.Get_Derived_Data(mocap_data,mocap_past_data[i],delta_time)

						#update past mocap data
						mocap_past_data[i]=mocap_data_derived

						#Publish data on topic
						topics_publisher[i].publish(mocap_data_derived)

			rate.sleep()

		utils.logwarn('Mocap stop publishing')


if __name__=="__main__":
	rospy.init_node("ros_mocap")
	Mocap()

#EOF