#!/usr/bin/env python

## Script that contain usefull function used in several packages
  
import rospy
import matplotlib
matplotlib.use("qt4agg")
import matplotlib.pyplot as plt 
#import threading
#let's try using multiprocessing instead of threading module:
import multiprocessing
import time
<<<<<<< HEAD
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure
=======
>>>>>>> stabilize-mode


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

def cbPlot(x,y):
    line = plt.plot(x,y)
    plt.show() #I think the code in the child will stop here until the graph is closed


def plot(x,y):
	multiprocessing.Process(target=cbPlot,args=(x,y)).start()

def pngplot(name,x,y):
	fig = Figure()
	canvas = FigureCanvas(fig)
	ax = fig.add_subplot(1,1,1)
	ax.plot(x,y)
	canvas.print_figure(name)