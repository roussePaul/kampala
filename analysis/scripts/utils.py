#!/usr/bin/env python
import rospy

"""This script contains usefull functions used in several packages"""
  
##@param msg: a message to be displayed as an error message
def logerr(msg):
  rospy.logerr(get_header()+'\033[91m'+str(msg)+'\033[0m')

##@param msg: a message to be displayed as a warning
def logwarn(msg):
  rospy.logwarn(get_header()+'\033[93m'+str(msg)+'\033[0m')

##@param msg: a message to be displayed as information
def loginfo(msg):
  rospy.loginfo(get_header()+'\033[0m'+str(msg))

##@return the header to be displayed 
def get_header():
  return '\033[92m\033[1m\t['+rospy.get_name()+'] ' 

##@param PARAMETER_NAME: the name of the parameter to be accessed
##@param DEFAULT_VALUE: the default value of the parameter
##@return the parameter obtained from the parameter server
def Get_Parameter(PARAMETER_NAME,DEFAULT_VALUE):
        """This function gets a certain parameter from the parameter server."""
	param=rospy.get_param(PARAMETER_NAME,DEFAULT_VALUE)
	if rospy.has_param(PARAMETER_NAME):
		loginfo(''+PARAMETER_NAME+' found: '+str(param))
	else:
		logwarn(''+PARAMETER_NAME+' not found. Default: '+str(DEFAULT_VALUE))

	return param
