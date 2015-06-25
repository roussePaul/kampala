#Vinzenz Minnig, 2015
#Contains the necessary function for the system setup while working with mavros and the qualysis motion capture system

import rospy
from mavros.srv import ParamSet
from mavros.srv import CommandBool
from mavros.srv import SetMode
from mocap.srv import BodyData
import sys


def Get_Parameter(NODE_NAME,PARAMETER_NAME,DEFAULT_VALUE):
	param=rospy.get_param(PARAMETER_NAME,DEFAULT_VALUE)
	if rospy.has_param(PARAMETER_NAME):
		rospy.loginfo('['+NODE_NAME+']: '+PARAMETER_NAME+' found: '+str(param))
	else:
		rospy.logwarn('['+NODE_NAME+']: '+PARAMETER_NAME+' not found. Default: '+str(DEFAULT_VALUE))

	return param


def Set_Flight_Mode(NODE_NAME,MODE):
	return_value=True

	#Change the flight mode on the Pixhawk flight controller
	try:
		rospy.wait_for_service('/mavros/set_mode',10)
	except:
		rospy.logerr('['+NODE_NAME+']: Mavros is not available')
		return_value=False

	rospy.loginfo('['+NODE_NAME+']: Changing flight mode to '+MODE+' ...')

	rospy.sleep(2)

	try:
		change_param=rospy.ServiceProxy('/mavros/set_mode',SetMode)
		param=change_param(0,MODE)
	except:
		rospy.logerr('['+NODE_NAME+']: Cannot change flight mode')
		return_value=False

	if param.success:
		rospy.loginfo('['+NODE_NAME+']: Flight mode changed to '+MODE)
	else:
		rospy.logerr('['+NODE_NAME+']: Cannot change flight mode')
		return_value=False

	return return_value


def Set_System_ID(NODE_NAME,id_int):
	return_value=True

	#Necesary to allow RCOverride
	#Also checks if there is a connection to Mavros, and shuts down if there isn't

	rospy.loginfo('['+NODE_NAME+']: Connecting to Mavros ...')
	try:
		rospy.wait_for_service('mavros/param/set',10)
	except:
		rospy.logerr('['+NODE_NAME+']: Mavros is not available')
		return_value=False
	rospy.loginfo('['+NODE_NAME+']: Connected to Mavros')

	rospy.loginfo('['+NODE_NAME+']: Changing system ID ...')

	rospy.sleep(2)

	try:
		change_param=rospy.ServiceProxy('mavros/param/set',ParamSet)
		param=change_param('SYSID_MYGCS',id_int,0.0)
	except:
		rospy.logerr('['+NODE_NAME+']: Cannot change system ID')
		return_value=False

	if param.success:
		rospy.loginfo('['+NODE_NAME+']: System ID changed')
	else:
		rospy.logerr('['+NODE_NAME+']: Cannot change system ID')
		return_value=False

	return return_value



def Connect_To_Mocap(NODE_NAME):
	#Connect to the Motion Capture System, flag an error if it is unavailable

	try:
		rospy.loginfo('['+NODE_NAME+']: Connecting to the mocap system...')
		rospy.wait_for_service('mocap_get_data',10)
	except:
		rospy.logerr('['+NODE_NAME+']: No connection to the mocap system')
		sys.exit()
	rospy.loginfo('['+NODE_NAME+']: Connected to Mocap')

	return rospy.ServiceProxy('mocap_get_data',BodyData)



def Arming_Quad(NODE_NAME):
	return_value=True

	#Arming the Quad
	try:
		rospy.loginfo('['+NODE_NAME+']: Arming Quad ...')
		rospy.wait_for_service('mavros/cmd/arming',10)
	except:
		rospy.logerr('['+NODE_NAME+']: No connection to Mavros')
		return_value=False

	try:
		arming=rospy.ServiceProxy('mavros/cmd/arming',CommandBool)
		arming_result=arming(True)
	except:
		rospy.logerr('['+NODE_NAME+']: Cannot arm quad')
		return_value=False

	rospy.sleep(1)

	#Arming has to be done twice sometimes...
	try:
		arming=rospy.ServiceProxy('mavros/cmd/arming',CommandBool)
		arming_result=arming(True)
	except:
		rospy.logerr('['+NODE_NAME+']: Cannot arm quad')
		return_value=False

	rospy.sleep(1)

	if arming_result.success:
		rospy.loginfo('['+NODE_NAME+']: Quad is now armed')
	else:
		rospy.logerr('['+NODE_NAME+']: Cannot arm quad')
		return_value=False

	return return_value

