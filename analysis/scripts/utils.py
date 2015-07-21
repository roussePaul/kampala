#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from threading import Thread
## Script that contain usefull function used in several packages
  

def logerr(msg):
  rospy.logerr(get_header()+'\033[91m'+str(msg)+'\033[0m')

def logwarn(msg):
  rospy.logwarn(get_header()+'\033[93m'+str(msg)+'\033[0m')

def loginfo(msg):
  rospy.loginfo(get_header()+'\033[0m'+str(msg))

def get_header():
  return '\033[92m\033[1m\t['+rospy.get_name()+'] ' 

def Get_Parameter(PARAMETER_NAME,DEFAULT_VALUE):
	param=rospy.get_param(PARAMETER_NAME,DEFAULT_VALUE)
	if rospy.has_param(PARAMETER_NAME):
		loginfo(''+PARAMETER_NAME+' found: '+str(param))
	else:
		logwarn(''+PARAMETER_NAME+' not found. Default: '+str(DEFAULT_VALUE))

	return param


class Plot(Thread):
    def __init__(self,x,y):
    	Thread.__init__(self)
    	self.x = x
    	self.y = y
    	self.start()

    def run(self):
	    plt.plot(self.x,self.y)
	    plt.show()