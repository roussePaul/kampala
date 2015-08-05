#!/usr/bin/env python
"""Useful scripts that don't really relate on any packages

Logging, plotting and debugging friendly functions.
"""

import rospy
import matplotlib
matplotlib.use("qt4agg")
import matplotlib.pyplot as plt 
import multiprocessing
import time
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure

## Log an error, print the node name before.
## @param msg: string ir stringable object 
def logerr(msg):
  rospy.logerr(get_header()+'\033[91m'+str(msg)+'\033[0m')

## Log a warning, print the node name before.
## @param msg: string ir stringable object
def logwarn(msg):
  rospy.logwarn(get_header()+'\033[93m'+str(msg)+'\033[0m')

## Log an information, print the node name before.
## @param msg: string ir stringable object
def loginfo(msg):
  rospy.loginfo(get_header()+'\033[0m'+str(msg))

## Get and colorize the current node name.
## Used in the log functions.
def get_header():
  return '\033[92m\033[1m\t['+rospy.get_name()+'] ' 

## Check if a ROS parameter exist, if so get it from rosparam, if not notice it as a warning.
## @param param_name: path of the ROS parameter (the same as the one you will use for the rospy.get_param() function)
## @param default_value: default value of the parameter to take if the parameter is not found
def Get_Parameter(param_name,default_value):
	param=rospy.get_param(param_name,default_value)
	if rospy.has_param(param_name):
		loginfo(''+param_name+' found: '+str(param))
	else:
		logwarn(''+param_name+' not found. Default: '+str(default_value))

	return param

## Plot data
## Blocking function used as a callback by the plot() function.
def cbPlot(x,y):
    line = plt.plot(x,y)
    plt.show() #I think the code in the child will stop here until the graph is closed

## Plot data in another thread
## @param x: x array data of the plot
## @param y: y array data of the plot
def plot(x,y):
	multiprocessing.Process(target=cbPlot,args=(x,y)).start()

## Plot data in a png image
## Since it is not possible to call ui stuff inside a service with ROS
## here is a little trick to get what we want, we print the graph in a picture instead of opening a gui outside of the rqt_gui part
## @param x: x array data of the plot
## @param y: y array data of the plot
def pngplot(name,x,y):
	fig = Figure()
	canvas = FigureCanvas(fig)
	ax = fig.add_subplot(1,1,1)
	ax.plot(x,y)
	canvas.print_figure(name)