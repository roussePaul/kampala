#!/usr/bin/env python
#Vinzenz Minnig, 2015

#imports the data from the Qualysis motion capture system into the ROS space
#data has to be requested by a service that this node provides
#the different services are:
#	get_available_bodies()		-> 	returns a list of avalable bodies by their id
#	get_data(body_id)			-> 	returns all the data from a specific body (body id passed as an argument)
#									the returned data is in the service format BodyData, which contains
#									x,y,z,pitch,roll,yaw

import sys
import rospy
import numpy

from mocap.msg import QuadPosition
from mocap.srv import Bodies
from mocap.srv import BodyData
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

from rotmat import Vector3, Matrix3
from math import radians, degrees


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

class Mocap:
	def __init__(self):
		self.bodies = []
		self.start_services()
		self.start_subscribes()
		rospy.spin()

	def get_bodies(self,arg):
		num_bodies = len(self.bodies.name)
		return {'list':range(num_bodies)}


	def get_data(self,body):
		data = QuadPosition()

		body_id = body.id

		data.found_body = False
		
		if self.bodies==[]:
			rospy.logerr("Gazebo not started")
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
		self.bodies = msg

	def start_services(self):
		gb=rospy.Service("mocap_get_bodies",Bodies,self.get_bodies)
		gd=rospy.Service("mocap_get_data",BodyData,self.get_data)

	def start_subscribes(self):
		rospy.Subscriber("/gazebo/model_states",ModelStates,self.update_positions)

if __name__=="__main__":
	rospy.init_node("ros_mocap")
	Mocap()

#EOF
