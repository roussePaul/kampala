#!/usr/bin/env python

#Take the quad data forwarded from the security guard
#Computes velocities and accelerations
#Forwards the data to the controller

import rospy
from mocap.msg import QuadPositionDerived


class Positioning():
	def __init__(self):
		self.x=0
		self.y=0
		self.z=0
		self.yaw=0
		self.x_vel=0
		self.y_vel=0
		self.z_vel=0
		self.yaw_vel=0
		self.x_acc=0
		self.y_acc=0
		self.z_acc=0
		self.yaw_acc=0

		self.x_prev=0
		self.v_prev=0
		self.z_prev=0
		self.yaw_prev=0
		self.x_vel_prev=0
		self.v_vel_prev=0
		self.z_vel_prev=0
		self.yaw_vel_prev=0

		self.time=rospy.Time.now()
		self.past_time=rospy.Time.now()
		self.time_diff=0

		self.got_point=False

	def time_difference(self):
		self.past_time=self.time
		self.time=rospy.Time.now()

		delta_time=self.time-self.past_time
		self.time_diff=delta_time.secs+(delta_time.nsecs/1E9)

		return self.time_diff

	def update_previous(self,x,y,z,yaw):
		self.x_prev=x
		self.y_prev=y
		self.z_prev=z
		self.yaw_prev=yaw

	def update_previous_vel(self,x_vel,y_vel,z_vel,yaw_vel):
		self.x_vel_prev=x_vel
		self.y_vel_prev=y_vel
		self.z_vel_prev=z_vel
		self.yaw_vel_prev=yaw_vel

	def set_pos(self,x,y,z,yaw):
		self.update_previous(self.x,self.y,self.z,self.yaw)
		self.x=x
		self.y=y
		self.z=z
		self.yaw=yaw

	def set_vel(self,x,y,z,yaw,time):
		self.update_previous_vel(self.x_vel,self.y_vel,self.z_vel,self.yaw_vel)
		self.x_vel=(x-self.x_prev)/time
		self.y_vel=(y-self.y_prev)/time
		self.z_vel=(z-self.z_prev)/time
		self.yaw_vel=(yaw-self.yaw_prev)/time


	def set_acc(self,x_vel,y_vel,z_vel,yaw_vel,time):
		self.x_acc=(x_vel-self.x_vel_prev)/time
		self.y_acc=(y_vel-self.y_vel_prev)/time
		self.z_acc=(z_vel-self.z_vel_prev)/time
		self.yaw_acc=(yaw_vel-self.yaw_vel_prev)/time



def Derivation(data,quad_state):
	quad_state.got_point=True
	time=quad_state.time_difference()
	
	quad_state.set_pos(data.x,data.y,data.z,data.yaw)
	quad_state.set_vel(quad_state.x,quad_state.y,quad_state.z,quad_state.yaw,time)
	quad_state.set_acc(quad_state.x_vel,quad_state.y_vel,quad_state.z_vel,quad_state.yaw_vel,time)



def Get_Augmented_Data(quad_state):
	data=QuadPositionDerived()
	data.x=quad_state.x
	data.y=quad_state.y
	data.z=quad_state.z
	data.yaw=quad_state.yaw
	data.x_vel=quad_state.x_vel
	data.y_vel=quad_state.y_vel
	data.z_vel=quad_state.z_vel
	data.yaw_vel=quad_state.yaw_vel
	data.x_acc=quad_state.x_acc
	data.y_acc=quad_state.y_acc
	data.z_acc=quad_state.z_acc
	data.yaw_acc=quad_state.yaw_acc
	data.time_diff=quad_state.time_diff

	return data



if __name__=='__main__':
	rospy.init_node('derivator')
	loop_rate=rospy.Rate(30)

	Quad_Pos=Positioning()

	#Subscribe to the forwarded data from the security guard
	rospy.Subscriber('/security_guard/data_forward',QuadPositionDerived,Derivation,Quad_Pos)

	#Publish to /derivator/pos_data for the controller
	data_to_controller=rospy.Publisher('/derivator/pos_data',QuadPositionDerived,queue_size=10)

	while not rospy.is_shutdown():

		if Quad_Pos.got_point:
			data_augmented=Get_Augmented_Data(Quad_Pos)
			data_to_controller.publish(data_augmented)
			Quad_Pos.got_point=False

		loop_rate.sleep()