#!/usr/bin/env python

## Convert the mavros interace so that it behave exactly like APM

import sys
import rospy

from mavros_msgs.msg import OverrideRCIn,BatteryStatus
from mavros_msgs.srv import ParamSet
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from px4.msg import manual_control_setpoint
import std_msgs.msg
import ctypes

## Class to create the suscrib to the rc/override topic and to publish it to the px4 simulated quadcopter. Create also the services.
class mavrosSim:
	## Constructor
	# Suscrib to the topics and create the publisher
	def __init__(self):
		self.px4_manual_control = rospy.Publisher('manual_control_setpoint',manual_control_setpoint, queue_size=10)
		rospy.Subscriber('mavros/rc/override',OverrideRCIn,self.transformMessage)
		rospy.Publisher('mavros/battery',BatteryStatus)
		self.param_set = rospy.Service('mavros/param/set', ParamSet, self.cb_param_set)
		self.set_mode = rospy.Service('mavros/set_mode', SetMode, self.cb_set_mode)
		self.arming = rospy.Service('mavros/cmd/arming', CommandBool, self.cb_arming)
		rospy.spin()

	## Emulate the services of parameter set
	def cb_param_set(self,msg):
		print 'Receive set param commande:'
		print msg
		return {'success':True, 'integer':0, 'real':0.0}

	## Emulate the services of mode set
	def cb_set_mode(self,msg):
		print 'Receive set mode commande:'
		print msg
		return {'success':True}

	## Emulate the services of arming
	def cb_arming(self,msg):
		self.pubRC(0, 0, 0, 1)
		rospy.sleep(0.1)
		self.pubRC(0, 0, 0, 0)
		return {'success':True, 'result':1}

	## Emulate the topic of rc override
	def transformMessage(self,msg):
		roll  	 = (msg.channels[0]-1500.0)/500.0
		pitch  	 = (msg.channels[1]-1500.0)/500.0
		thrust   = (msg.channels[2]-1000.0)/1000.0
		yaw_rate = (msg.channels[3]-1500.0)/500.0

		self.pubRC(roll, pitch, thrust, yaw_rate)

		## Publish RC data to the PX4 simulation
	def pubRC(self, roll, pitch, thrust, yaw_rate):
		pub = manual_control_setpoint()
		pub.timestamp = 0
		pub.x = -pitch
		pub.y = roll
		pub.z = thrust
		pub.r = yaw_rate
		pub.mode_switch     = manual_control_setpoint.SWITCH_POS_OFF
		pub.return_switch   = manual_control_setpoint.SWITCH_POS_OFF
		pub.posctl_switch   = manual_control_setpoint.SWITCH_POS_OFF
		pub.loiter_switch   = manual_control_setpoint.SWITCH_POS_OFF
		pub.offboard_switch = manual_control_setpoint.SWITCH_POS_OFF
		pub.acro_switch     = manual_control_setpoint.SWITCH_POS_NONE

		self.px4_manual_control.publish(pub)

if __name__=="__main__":
	rospy.init_node("mavros_sim")
	mavrosSim()

#EOF
