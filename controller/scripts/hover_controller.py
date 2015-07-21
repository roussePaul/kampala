#!/usr/bin/env python

#Basic hovering controller 
#The parameters are the body ID from Qualysis and the hovering height


import rospy
from controller.msg import PlotData
from mavros.msg import OverrideRCIn
from mavros.srv import ParamSet
from mocap.srv import BodyData
import sys
import math
import sml_setup

import analysis
import utils

#Constants
#*************************************
NODE_NAME='HC'

CONTROL_MIN=1000
CONTROL_NEUTRAL=1500
CONTROL_MAX=2000
CONTROL_ARMING_MIN=1021
CONTROL_CANCEL_GRAVITY=1545 #Around 1565, but to be on the safe side...

RELEASE=0

DEFAULT_BODY_ID=21
DEFAULT_HOVERING_HEIGHT=0.6
#yaw angle goes from -180 to 180 degrees
YAW_ANGLE=0

#Discard value that are x times higher than the median of the previous measures
MEDIAN_FILTER_THRESHOLD=5

#Controller parameters
Ktt=1000/(20*math.pi/180)
Kphi=1000/(20*math.pi/180)

w=2
x_i=math.sqrt(2)/2
Kp=w*w
Kv=2*x_i*w

N_yaw=500
K_yaw=2
w_inf=5

I_lim=0.5
K_i=7
#*************************************

def Control_To_RC(channel):
	data=OverrideRCIn()
	#Release all channels back to RC
	command=[RELEASE,RELEASE,RELEASE,RELEASE,RELEASE,RELEASE,RELEASE,RELEASE]
	data.channels=command
	channel.publish(data)
	rospy.logwarn('['+NODE_NAME+']: Returning control to RC')

	return


def Get_Current_State(body,body_prev,time):
	#Get the current position and velocity of the quad

	x1=body.pos.x
	x2=body.pos.y
	x3=body.pos.z
	x4=body.pos.yaw

	delta_x1=body.pos.x-body_prev.pos.x
	delta_x2=body.pos.y-body_prev.pos.y
	delta_x3=body.pos.z-body_prev.pos.z
	delta_x4=body.pos.yaw-body_prev.pos.yaw

	v1=delta_x1/time
	v2=delta_x2/time
	v3=delta_x3/time
	v4=delta_x4/time

	#Add a filter on the velocity to cancel out numerical spikes
	#(v1,v2,v3,v4)=MedianFilter(??)

	return (x1,x2,x3,x4),(v1,v2,v3,v4)


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


def Hovering(x,v,x_target,delta_t,current_d):
	#throttle, pitch and roll:
	u=[]
	AUX=[]
	AUX_rot=[]

	new_d=current_d+delta_t*(K_i*(((x[2]-x_target[2])*Kv/2)+v[2]))
	new_d=Saturation(new_d,-I_lim,I_lim)

	for i in range(0,3):
		u.append(-Kv*v[i]-Kp*(x[i]-x_target[i]))

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

	#Implement some saturation
	pitch=Saturation(pitch,1400,1600)
	roll=Saturation(roll,1400,1600)

	return [roll,pitch,throttle,yaw,0,0,0,0],new_d



if __name__=='__main__':
	rospy.init_node('hovering_controller')

	#Get parameters body_id and hovering target height
	body_id=utils.Get_Parameter(NODE_NAME,'hover_controller/qualysis_id',DEFAULT_BODY_ID)
	hovering_alt=utils.Get_Parameter(NODE_NAME,'hover_controller/height',DEFAULT_HOVERING_HEIGHT)

	loop_rate=rospy.Rate(30)

	#Set the system ID to allow RC Override
	#sml_setup.Set_System_ID(NODE_NAME)

	#Publish to RC Override
	rc_override=rospy.Publisher('mavros/rc/override',OverrideRCIn,queue_size=10)
	#Publish for rqt_graph to show the positions of the quad
	plot_pos_x=rospy.Publisher('/controller/plot_data_x',PlotData,queue_size=10)
	plot_pos_y=rospy.Publisher('/controller/plot_data_y',PlotData,queue_size=10)
	plot_pos_z=rospy.Publisher('/controller/plot_data_z',PlotData,queue_size=10)

	#Set up service to request body data from Qualysis
	body_info=sml_setup.Connect_To_Mocap(NODE_NAME)

	data=OverrideRCIn()
	command=[0,0,0,CONTROL_ARMING_MIN,0,0,0,0]
	data.channels=command
	rc_override.publish(data)

	rospy.loginfo('['+NODE_NAME+']: Hovering at '+str(hovering_alt)+' meters...')

	#Get initial body position
	try:
		body=body_info(body_id)
		body_prev=body
	except:
		rospy.logerr('['+NODE_NAME+']: No connection to the mocap system')
		Control_To_RC(rc_override)
		sys.exit()

	#Get current time
	current_time=rospy.Time.now()
	past_time=current_time

	#define target position vector, take current x and y positions as target
	#(x,y,z,yaw[degrees])
	#x_target=(body.pos.x,body.pos.y,hovering_alt,YAW_ANGLE)
	x_target=(0,0,hovering_alt,YAW_ANGLE)

	d_updated=0

	while not rospy.is_shutdown():

		#Get coordinates from Qualysis
		try:
			body_prev=body
			body=body_info(body_id)
		except:
			rospy.logerr('['+NODE_NAME+']: No connection to the mocap system')
			Control_To_RC(rc_override)
			sys.exit()

		#Get time difference since last measurement
		past_time=current_time
		current_time=rospy.Time.now()
		delta_time=current_time-past_time
		time_diff=delta_time.secs+(delta_time.nsecs/1E9)

		#compute current position and velocity
		x,v=Get_Current_State(body,body_prev,time_diff)

		#implement the controller
		command_controlled,d_updated=Hovering(x,v,x_target,time_diff,d_updated)

		data.channels=command_controlled
		rc_override.publish(data)



		#Publish data for rqt_plot
		x_plot=PlotData()
		x_plot.target=x_target[0]
		x_plot.quad_pos=x[0]
		plot_pos_x.publish(x_plot)

		y_plot=PlotData()
		y_plot.target=x_target[1]
		y_plot.quad_pos=x[1]
		plot_pos_y.publish(y_plot)

		z_plot=PlotData()
		z_plot.target=x_target[2]
		z_plot.quad_pos=x[2]
		plot_pos_z.publish(z_plot)



		loop_rate.sleep()


#EOF