#!/usr/bin/env python
#Vinzenz Minnig, 2015

# mavros to px4

import sys
import rospy

from mavros.msg import OverrideRCIn
from mavros.srv import ParamSet
from mavros.srv import CommandBool
from mavros.srv import SetMode
from px4.msg import manual_control_setpoint
import std_msgs.msg
import ctypes

class mavrosSim:
	def __init__(self):
		self.px4_manual_control = rospy.Publisher('manual_control_setpoint',manual_control_setpoint, queue_size=10)
		rospy.Subscriber('mavros/rc/override',OverrideRCIn,self.transformMessage)
		self.param_set = rospy.Service('mavros/param/set', ParamSet, self.cb_param_set)
		self.set_mode = rospy.Service('mavros/set_mode', SetMode, self.cb_set_mode)
		self.arming = rospy.Service('mavros/cmd/arming', CommandBool, self.cb_arming)
		rospy.spin()

	def cb_param_set(self,msg):
		print 'Receive set param commande:'
		print msg
		return {'success':True, 'integer':0, 'real':0.0}

	def cb_set_mode(self,msg):
		print 'Receive set mode commande:'
		print msg
		return {'success':True}

	def cb_arming(self,msg):
		self.pubRC(0, 0, 0, 1)
		rospy.sleep(0.1)
		self.pubRC(0, 0, 0, 0)
		return {'success':True, 'result':1}

	def transformMessage(self,msg):
		roll  	 = (msg.channels[0]-1500.0)/500.0
		pitch  	 = (msg.channels[1]-1500.0)/500.0
		thrust   = (msg.channels[2]-1000.0)/1000.0
		yaw_rate = (msg.channels[3]-1500.0)/500.0

		self.pubRC(roll, pitch, thrust, yaw_rate)

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
