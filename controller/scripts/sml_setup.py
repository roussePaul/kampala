#Vinzenz Minnig, 2015
"""This script contains the necessary function for the system setup while working with mavros and the qualysis motion capture system."""

import rospy
from mavros.srv import ParamSet, ParamGet
from mavros.srv import CommandBool
from mavros.srv import SetMode
from mocap.srv import BodyData
import sys

import analysis
import utils

##@param MODE: a string representing the flight mode to be used
##@return true if the flight mode was set successfully, false otherwise
def Set_Flight_Mode(MODE):
        """This function sets the flight mode of the drone to MODE."""
	return_value=True

	#Change the flight mode on the Pixhawk flight controller
	try:
		rospy.wait_for_service('mavros/set_mode',10)
	except:
		utils.logerr('Mavros is not available')
		return_value=False

	utils.loginfo('Changing flight mode to '+MODE+' ...')

	rospy.sleep(2)

	try:
		change_param=rospy.ServiceProxy('mavros/set_mode',SetMode)
		param=change_param(0,MODE)
		while param.success==False:
			param=change_param(0,MODE)
			if param.success==False:
				utils.logerr('Cannot change flight mode')
		if param.success:
			utils.loginfo('Flight mode changed to '+MODE)
		else:
			utils.logerr('Cannot change flight mode')
			return_value=False
	except:
		utils.logerr('Cannot change flight mode')
		return_value=False


	return return_value

##@param id_int: an integer to set the system ID to
##@return true if the system ID was set successfully and a connection to Mavros was establishe, false otherwise
def Set_System_ID(id_int):
	return_value=True
        """This function sets the system ID and checks if it is possible to connect to mavros. The system ID
        should be 1 to be able to use the rc/override topic."""

	utils.loginfo('Connecting to Mavros ...')
	try:
		rospy.wait_for_service('mavros/param/set',10)
	except:
		utils.logerr('Mavros is not available (fail to contact service mavros/param/set).')
		return_value=False
	utils.loginfo('Connected to Mavros')

	utils.loginfo('Changing system ID ...')

	rospy.sleep(2)

	try:
		change_param=rospy.ServiceProxy('mavros/param/set',ParamSet)
		param=change_param('SYSID_MYGCS',id_int,0.0)

		if param.success:
			utils.loginfo('System ID changed')
		else:
			utils.logerr('Cannot change system ID')
			return_value=False
	except:
		utils.logerr('Cannot change system ID')
		return_value=False


	return return_value

##@param id_int: the system ID
##return true if the system id was changed and the connection to Mavros established, false otherwise
def Wait_For_ID(id_int):
	return_value=False

	# Wait for Mavros connection and wait until the parameter SYSID_MYGCS is available

	utils.loginfo('Connecting to Mavros ...')
	try:
		rospy.wait_for_service('mavros/param/set',10)
	except:
		utils.logerr('Mavros is not available (fail to contact service mavros/param/set).')
		return_value=False
	utils.loginfo('Connected to Mavros')

	set_param=rospy.ServiceProxy('mavros/param/set',ParamSet)
	while return_value==False:
		try:
			param=set_param('SYSID_MYGCS',id_int,0.0)
			if param.success:
				utils.loginfo('System ID changed')
				return_value=True
			else:
				utils.logerr('Cannot change system ID')
				return_value=False
		except:
			utils.logerr('Cannot change system ID')
			return_value=False
		rospy.sleep(5)

	utils.loginfo('Iris initialised...')
	return return_value

##@return returns true if the quad was armed, false otherwise
def Wait_For_Arming():
	return_value=False

	#Arming the Quad
	try:
		utils.loginfo('Arming Quad ...')
		rospy.wait_for_service('mavros/cmd/arming',10)
	except:
		utils.logerr('No connection to Mavros')
		return_value=False

	try:
		arming=rospy.ServiceProxy('mavros/cmd/arming',CommandBool)
		arming_result=arming(True)
	except:
		utils.logerr('Cannot arm quad')
		return_value=False

	rospy.sleep(1)


	set_param=rospy.ServiceProxy('mavros/param/set',ParamSet)
	while return_value==False:
		#Arming has to be done twice sometimes...
		try:
			arming=rospy.ServiceProxy('mavros/cmd/arming',CommandBool)
			arming_result=arming(True)
			if arming_result.success:
				utils.loginfo('Quad is now armed')
			else:
				utils.logerr('Cannot arm quad')
				return_value=False
		except:
			utils.logerr('Cannot arm quad')
			return_value=False
		rospy.sleep(20)


	return return_value

def Connect_To_Mocap_Message():
	utils.loginfo('Connecting to Mocap system')
	return

##@return a service proxy to get the motion capture data
def Connect_To_Mocap():
	""" This function connects to the motion capture system and flags an error if it is unavailable."""

	try:
		utils.loginfo('Connecting to the mocap system...')
		rospy.wait_for_service('mocap_get_data',10)
	except:
		utils.logerr('No connection to the mocap system')
		sys.exit()
	utils.loginfo('Connected to Mocap')

	return rospy.ServiceProxy('mocap_get_data',BodyData)


##@return returns true if the quad was armed successfully and false otherwise
def Arming_Quad(base_name=""):
        """This function is used to arm the quad."""
	return_value=True

	#Arming the Quad
	srv_path = 'mavros/cmd/arming'
	if base_name!="":
		srv_path = "/%s/%s"%(base_name,srv_path)

	try:
		utils.loginfo('Arming Quad ...')
		rospy.wait_for_service(srv_path,10)
	except:
		utils.logerr('No connection to Mavros')
		return_value=False

	try:
		arming=rospy.ServiceProxy(srv_path,CommandBool)
		arming_result=arming(True)
	except:
		utils.logerr('Cannot arm quad')
		
		return_value=False

	rospy.sleep(1)

	#Arming has to be done twice sometimes...
	try:
		arming=rospy.ServiceProxy(srv_path,CommandBool)
		arming_result=arming(True)
	except:
		utils.logerr('Cannot arm quad')
		return_value=False

	rospy.sleep(1)

	if arming_result.success:
		utils.loginfo('Quad is now armed')
	else:
		utils.logerr('Cannot arm quad')
		return_value=False

	return return_value

