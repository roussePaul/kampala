#!/usr/bin/env python

#Security feature of autonomous drone flight in the SML lab
#Checks the data coming from Qualysis, as well as the safety boundaries set for the drone
#In case of violations, disables the controller, and enables the landing node

"""This script provides the security guard that gives permission to either the controller or 
the lander to publish commands to the drone. Which of the two is the case depends on whether or not
the drone can be found by the motion capture system and whether or not it is within the virtual 
safety zone."""

import rospy
import sml_setup
import sys
from mocap.msg import QuadPositionDerived
from controller.msg import Permission

import analysis
import utils


class Trajectory():
        """This is a simple class to check if a trajectory is done."""
	def __init__(self):
		self.is_done=False


class Point():
        """The functionality of this class is similar to that of the other point classes.
        Extra functionality is provided concerning timing. This is necessary to identify
        whether or not there is a problem with the motion capture system."""
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
		self.time=rospy.Time.now()
		self.time_secs=0
 
	def update_time(self):
		time_now=rospy.Time.now()
		time_diff=time_now-self.time
		self.time_secs=time_diff.secs+(time_diff.nsecs/1E9)
		self.time=rospy.Time.now()
		return(self.time_secs)

	def get_time(self):
		time_now=rospy.Time.now()
		time_diff=time_now-self.time
		self.time_secs=time_diff.secs+(time_diff.nsecs/1E9)
		return(self.time_secs)

##@param lander_channel: the publisher publishing permission to the lander
##@param controller_channer: the publisher publishing permission to the controller
def Interrupt_Flight(lander_channel,controller_channel):
	"""This function is used to interrupt the flight by giving permission to the lander and
        disabling the controller."""
	rate=rospy.Rate(30)
	utils.logerr('Flight interrupted, landing mode active')
	while not rospy.is_shutdown():
		lander_channel.publish(True)
		controller_channel.publish(False)
		rate.sleep()

##@return true if the quad was successfully armed, false otherwise
def Prepare_For_Flight():
        """This function sets the flight mode of the quad, sets the system ID to allow RC override and
        arms the quad."""
	mode_success=sml_setup.Set_Flight_Mode('STABILIZE')
	ID_success=sml_setup.Set_System_ID(1)
	arming_success=sml_setup.Arming_Quad()
	if mode_success and ID_success and arming_success:
		return True
	else:
		return False



##@param x: the x-component of the position of the quad
##@param y: the y-component of the position of the quad
##@param z: the z-component of the position of the quad
##@return true if the quad is within the boundaries, false otherwise
def Within_Boundaries(x,y,z):
	"""This function checks whether or not the quad is within the virtual safety area defined in 
        the launch file."""
	shape = rospy.get_param('security_guard/shape','cube')
	centerx = float(rospy.get_param('security_guard/centerx',0.0))
	centery = float(rospy.get_param('security_guard/centery',0.0))

	if shape == 'cube':
		side = float(rospy.get_param('security_guard/side',4.0))
		return x>=(centerx - side/2) and x<=(centerx + side/2) and y>=(centery - side/2) and y<=(centery + side/2) and z<=side

	radius = float(rospy.get_param('security_guard/radius',0))
	if shape == 'hemisphere':
		return ((x-centerx)**2 + (y-centery)**2 + z**2)<=radius**2
	height = float(rospy.get_param('security_guard/height',0))
	if shape == 'cylinder':
		return ((x-centerx)**2 + (y-centery)**2)<=radius**2 and z<=height
	return False


  

##@param current_point: the current point object associated with the quad
##@return true if the quad is still tracked by the motion capture system, false otherwise
def Security_Check(current_point):
        """This function checks whether or not the quad is found by the motion capture system."""
	keep_controller=False

	if current_point.get_time()<0.5:
		if current_point.found_body:
			if Within_Boundaries(current_point.x,current_point.y,current_point.z):
				keep_controller=True
			else:
				utils.logerr('Out of boundaries.')
				keep_controller=False
				return keep_controller
		else:
			utils.logerr('Body not found.')
			keep_controller=False
			return keep_controller
	else:
		utils.logerr('Lost mocap signal (no signal during more than 0.5 seconds).')
		keep_controller=False
		return keep_controller

	return keep_controller

##@param data: an instance of the class Permission defined by the corresponding ROS msg
##@param end_trajectory: an instance of the class Trajectory defined in this script
def Trajectory_Done(data,end_trajectory):
        """This function checks if the trajectory is completed. If it is the drone lands."""
	if data.permission:
		if not end_trajectory.is_done:
			utils.loginfo('Trajectory is completed')
		end_trajectory.is_done=True

##@param data: a point object
##@param pont_obj: a point object
def New_Point(data,point_obj):
        """This function sets the point object point_obj to the point object data."""
	point_obj.found_body=data.found_body
	point_obj.x=data.x
	point_obj.y=data.y
	point_obj.z=data.z
	point_obj.yaw=data.yaw
	point_obj.pitch=data.pitch
	point_obj.roll=data.roll
	point_obj.x_vel=data.x_vel
	point_obj.y_vel=data.y_vel
	point_obj.z_vel=data.z_vel
	point_obj.yaw_vel=data.yaw_vel
	point_obj.pitch_vel=data.pitch_vel
	point_obj.roll_vel=data.roll_vel
	point_obj.x_acc=data.x_acc
	point_obj.y_acc=data.y_acc
	point_obj.z_acc=data.z_acc
	point_obj.yaw_acc=data.yaw_acc
	point_obj.pitch_acc=data.pitch_acc
	point_obj.roll_acc=data.roll_acc
	point_obj.update_time()
	return

##@param obj: a point object
##@return the QuadPositionDerived object corresponding to the point object obj
def Get_Quad_State(obj):
	result=QuadPositionDerived()
	result.found_body=obj.found_body
	result.x=obj.x
	result.y=obj.y
	result.z=obj.z
	result.yaw=obj.yaw
	result.pitch=obj.pitch
	result.roll=obj.roll
	result.x_vel=obj.x_vel
	result.y_vel=obj.y_vel
	result.z_vel=obj.z_vel
	result.yaw_vel=obj.yaw_vel
	result.pitch_vel=obj.pitch_vel
	result.roll_vel=obj.roll_vel
	result.x_acc=obj.x_acc
	result.y_acc=obj.y_acc
	result.z_acc=obj.z_acc
	result.yaw_acc=obj.yaw_acc
	result.pitch_acc=obj.pitch_acc
	result.roll_acc=obj.roll_acc
	result.time_diff=obj.time_secs
	return(result)


if __name__=='__main__':
	rospy.init_node('security_guard_topic')
	loop_rate=rospy.Rate(30)
	current_point=Point()

	controller_on=True
	lander_permission=Permission()
	controller_permission=Permission()
	trajectory_done=Trajectory()

	#Get the body ID as a parameter
	body_id=utils.Get_Parameter('body_id',8)
	mocap_topic='/body_data/id_'+str(body_id)

	#Publish topics
	lander_channel=rospy.Publisher('security_guard/lander',Permission,queue_size=10)
	controller_channel=rospy.Publisher('security_guard/controller',Permission,queue_size=10)
	data_forward=rospy.Publisher('security_guard/data_forward',QuadPositionDerived,queue_size=10)

	#Subscribe topics
	rospy.Subscriber(mocap_topic,QuadPositionDerived,New_Point,current_point)
	rospy.Subscriber('trajectory_gen/done',Permission,Trajectory_Done,trajectory_done)

	#Connect to Qualysis Motion Capture System
	body_info=sml_setup.Connect_To_Mocap_Message()


	#Prepare the Iris for flight (set system ID and arm)
	ready_to_fly=Prepare_For_Flight()

	if not ready_to_fly:
		Interrupt_Flight(lander_channel,controller_channel)

	
	while not rospy.is_shutdown():
		controller_on=Security_Check(current_point)
		if trajectory_done.is_done:
			controller_on=False

		if controller_on:
			lander_permission.permission=False
			controller_permission.permission=True
		else:
			utils.logerr('Initiate landing mode')

			lander_permission.permission=True
			controller_permission.permission=False
			while not rospy.is_shutdown():
				lander_channel.publish(lander_permission)
				controller_channel.publish(controller_permission)
				loop_rate.sleep()

		lander_channel.publish(lander_permission)
		controller_channel.publish(controller_permission)
		if controller_on:
			quad_state=Get_Quad_State(current_point)
			data_forward.publish(quad_state)

		loop_rate.sleep()


#EOF
