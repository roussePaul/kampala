#!/usr/bin/env python

import rospy
import sys
import math

from mavros.msg import BatteryStatus
from std_srvs.srv import Empty

import analysis
import utils

import numpy as np
from scipy.interpolate import Rbf
from scipy.interpolate import griddata
from scipy.interpolate import NearestNDInterpolator
import copy
import gnosis.xml.pickle

from controller.srv import PlotLAC 

## Represente a vector of inputs and the associated ROS parameters of the PID controller.
## 
## @var params_name: list of all the controller parameters that will be adapted.
class Point:
  params_name = ["CONTROL_CANCEL_GRAVITY"]

  def __init__(self):  
    self.params_value = dict()

  ## Read ros parameters
  def read_param(self):
    self.params_value = dict()
    for p in Point.params_name:
      self.params_value[p] = float(rospy.get_param(p))

  ## Write ROS parameters
  def write_param(self):
    for p in self.params_name:
      rospy.set_param(p,self.params_value[p])

  ## set the value of this point
  def set_inputs(self,inputs):
    self.inputs = copy.deepcopy(inputs)

## Linear adaptative controller.
##
## [Wikipedia page](https://en.wikipedia.org/wiki/Adaptive_control).
##
## This node try to conteract the unmodelled effects (such as the battery voltage) by adapting the parameters of the PID controller.
## The current implementation needs to be used with the LAC plugin.
##
## Support multiple inputs.
## But in this current version, just the battery level is used.
##
## The main difficulty of this class was to extend it to multiple dimensions 
## (if it was easy to find a library to perform the interpolation between the severals registered points, it was not for the extrapolation).
class LinearAC:
  def __init__(self, filename):
    rospy.init_node("LinearAC")
    self.filename = filename
    self.load()
    self.init_services_proxy()
    self.init_inputs()
    rospy.spin()

  ## Init the services
  def init_services_proxy(self):
    try: 
      self.params_load = rospy.ServiceProxy("blender/update_parameters", Empty)
      self.params_load_PID = rospy.ServiceProxy("PID_controller/update_parameters", Empty)
    except rospy.ServiceException as exc:
      utils.loginfo("PID not reachable " + str(exc))
    
    rospy.Service('LinearAC/add_point', Empty, self.add_point)
    rospy.Service('LinearAC/update_controller', Empty, self.update_current_point)
    rospy.Service('LinearAC/save', Empty, self.cbSave)
    rospy.Service('LinearAC/load', Empty, self.cbLoad)
    rospy.Service('LinearAC/plot', PlotLAC, self.cbPlot)
    rospy.Service('LinearAC/print', Empty, self.cbPrint)

  ## Compute the new PID parameters associated from the current inputs.
  ##
  ## Callback of the LinearAC/update_controller service.
  ##
  ## @param msg: Empty message.
  def update_current_point(self,msg):
    current_inputs = self.get_inputs()

    point_update = self.interpolate_point(current_inputs)

    point_update.write_param()

    self.params_load()
    self.params_load_PID()
    return []

  ## Perform the multi-input interpolation/extrapolation.
  ##
  ## If the point is inside the convex hull, this perform a linear interpolation.
  ## If the point is outside the convex hull, this function return 
  ## \f[ \mathlarger{\sum_{i=1}^{N} y_i \, e^{-\frac{\left \| x-x_i  \right \|^2}{l}}}\f],
  ## where \f$x\f$ is the input point, and \f$\left ( x_i,y_i \right )\f$ are the registered points of inputs and PID parameters.
  ## 
  ##  @param inputs_grid: array of registered points
  ##  @param values: array of parameters
  ##  @param inputs: input point we want to compute
  ##  @param l: \f$l\f$ parameter used for the extrapolation
  def inter_extrapolate(self,inputs_grid,values,inputs,l):
    if len(values)==1:
      val = values[0].item()
    else:
      val = float('nan')
      if len(values)>2:
        val = griddata(inputs_grid,values,inputs, method="linear")
        val = val.item()
      if np.isnan(val):
        d = []
        sum_v = 0.0
        sum_d = 0.0
        for (x,v) in zip(inputs_grid,values):
          d = np.linalg.norm(x-inputs)
          if d==0.0:
            return v.item()
            break
          f = math.exp(-d**2/l)
          sum_v += v*f
          sum_d += f

        val = sum_v/sum_d

        val = val.item()

    return val

  ## Execute the parameter comuptation for every parameter of the Point.params_name
  ##
  ## @param current_inputs: current inputs of the LAC block (ex: battery voltage)
  def interpolate_point(self, current_inputs):
    pts = Point()
    
    inputs_grid = [p.inputs for p in self.points]
    
    new_param = dict()

    for p_name in Point.params_name:
      values = [p.params_value[p_name] for p in self.points]
      val = self.inter_extrapolate(np.array(inputs_grid),np.array(values),tuple(current_inputs), 0.5)
      new_param[p_name] = val

    pts.params_value = new_param

    pts.set_inputs(current_inputs)

    return pts

  ## Initialize all the topics of inputs
  def init_inputs(self):
    rospy.Subscriber('mavros/battery', BatteryStatus, self.cbBattery)
    self.battery_value = 12.0

  ## Get the value og the battery input
  def cbBattery(self,msg):
    self.battery_value = msg.voltage

  ## Get the array of inputs
  def get_inputs(self):
    return [self.battery_value]

  ## Register a new points with the current inputs and the currents parameters
  def add_point(self,msg):
    point = Point()
    
    point.read_param()
    
    inputs = self.get_inputs()
    point.set_inputs(inputs)

    self.points.append(point)
    return []

  ## Callback of the save service
  def cbSave(self,msg):
    self.save()
    return []

  ## Callback of the load service
  def cbLoad(self,msg):
    self.load()
    return []

  ## Callback of the plot service
  def cbPlot(self,msg):
    inputs = self.get_inputs()

    values = [p.params_value[msg.plot_variable] for p in self.points]

    vmin = min(values)-1
    vmax = max(values)+1
    x = np.linspace(vmin,vmax,100)
    y = np.linspace(vmin,vmax,100)
    
    for i in range(0,100):
      inputs[msg.input_variable] = x[i]
      pts = self.interpolate_point(inputs)
      y[i] = pts.params_value[msg.plot_variable]

    utils.plot(copy.deepcopy(x),copy.deepcopy(y))
    return []

  ## Callback of the print service
  def cbPrint(self,msg):
    inputs_grid = [p.inputs for p in self.points]
    utils.loginfo(inputs_grid)
    for p_name in Point.params_name:
      values = [p.params_value[p_name] for p in self.points]
      utils.loginfo(p_name + " : " + str(values))

    return []

  ## Save the current state of the LAC node
  def save(self):
    with open(self.filename, 'w') as output:
      utils.loginfo("Save")
      outxml = gnosis.xml.pickle.dumps(self.points)
      output.write(outxml)

  ## Load the state of the LAC node
  def load(self):
    if self.filename != "":
      utils.loginfo("Loading file: "+self.filename)
      try:
        with open(self.filename, 'r') as input:
          utils.loginfo("File successfully loaded.")
          self.points = gnosis.xml.pickle.loads(input.read())
        return
      except IOError:
        self.newfile()
        return 

  ## Create a new file
  def newfile(self):
      utils.loginfo("File \"%s\" does not exist!"%(self.filename))
      self.points = []

if __name__ == '__main__':
  LinearAC(sys.argv[1])


#EOF
