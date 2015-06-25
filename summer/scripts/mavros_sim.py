#!/usr/bin/env python
#Vinzenz Minnig, 2015

#imports the data from the Qualysis motion capture system into the ROS space
#data has to be requested by a service that this node provides
#the different services are:
#   get_available_bodies()      ->  returns a list of avalable bodies by their id
#   get_data(body_id)           ->  returns all the data from a specific body (body id passed as an argument)
#                                   the returned data is in the service format BodyData, which contains
#                                   x,y,z,pitch,roll,yaw

import sys
import rospy

from mavros.msg import OverrideRCIn
from mavros.srv import ParamSet
from mavros.srv import CommandBool
from mavros.srv import SetMode
from mav_msgs.msg import CommandRollPitchYawrateThrust
import std_msgs.msg

class mavrosSim:
	def __init__(self):
		self.controller = rospy.Publisher('command/roll_pitch_yawrate_thrust', CommandRollPitchYawrateThrust, queue_size=10)
		rospy.Subscriber('mavros/rc/override',OverrideRCIn,self.transformMessage)
		self.param_set = rospy.Service('mavros/param/set', ParamSet, self.cb_param_set)
		self.set_mode = rospy.Service('mavros/set_mode', SetMode, self.cb_set_mode)
		self.arming = rospy.Service('mavros/cmd/arming', CommandBool, self.cb_arming)
		rospy.spin()

	def cb_param_set(self,msg):
		print msg
		return {'success':True, 'integer':0, 'real':0.0}

	def cb_set_mode(self,msg):
		print msg
		return {'success':True}

	def cb_arming(self,msg):
		print msg
		return {'success':True, 'result':1}

	def transformMessage(self,msg):
		h = std_msgs.msg.Header()
		h.stamp = rospy.Time.now()
		
		pub = CommandRollPitchYawrateThrust()
		pub.header = h

		pub.roll  	 = 0.1*(msg.channels[0]-1500.0)/500.0
		pub.pitch  	 = -0.1*(msg.channels[1]-1500.0)/500.0
		pub.thrust   = 30*(msg.channels[2]-1000.0)/1000.0
		pub.yaw_rate = 100*(msg.channels[3]-1500.0)/500.0
		self.controller.publish(pub)

if __name__=="__main__":
	rospy.init_node("mavros_sim")
	mavrosSim()

#EOF
