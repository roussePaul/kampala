#!/usr/bin/env python

#Provides a service giving the position of an object and its velocities and accelerations
#Makes use of the services provided by the ROS_Mocap node

import rospy
import sml_setup
from mocap.msg import QuadPositionDerived
from controller.srv import BodyDataDerived

#**********Constants*************
NODE_NAME="MD"
#********************************

def Derive(current,previous,delta_t):
	result=QuadStateSimple()
	result.x=(current.x-previous.x)/delta_t
	result.y=(current.y-previous.y)/delta_t
	result.z=(current.z-previous.z)/delta_t
	result.yaw=(current.yaw-previous.yaw)/delta_t

	return result


def MakeQuadState(pos,vel,acc,time):
	result=QuadPositionDerived()
	result.x=pos.x
	result.y=pos.y
	result.z=pos.z
	result.yaw=pos.yaw
	result.x_vel=vel.x
	result.y_vel=vel.y
	result.z_vel=vel.z
	result.yaw_vel=vel.yaw
	result.x_acc=acc.x
	result.y_acc=acc.y
	result.z_acc=acc.z
	result.yaw_acc=acc.yaw
	result.time_diff=time

	return result


def Callback(input_data):
	body_id=input_data.id
	delta_t=input_data.time_diff
	pos_previous=input_data.past_positions
	vel_previous=input_data.past_velocities

	try:
		body=body_info(body_id)
	except:
		return False, QuadPositionDerived()

	pos_now=QuadPositionDerived(body.pos.x,body.pos.y,body.pos.z,body.pos.yaw)
	vel_now=Derive(pos_now,pos_previous,delta_t)
	acc_now=Derive(vel_now,vel_previous,delta_t)

	State=MakeQuadState(pos_now,vel_now,acc_now,delta_t)

	return body.pos.found_body,State



if __name__=='__main__':
	rospy.init_node('mocap_derivator')

	body_info=sml_setup.Connect_To_Mocap(NODE_NAME)

	Mocap_Derivator=rospy.Service("mocap_get_data_derived",BodyDataDerived,Callback)

	while not rospy.is_shutdown():
		rospy.spin()