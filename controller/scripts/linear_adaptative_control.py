#!/usr/bin/env python

# PID controller for the IRIS+ in the SML Lab 
# Gets inputs from the Security Guard and the Trajectory Generator
# Publishes commands via Mavros' rc/override topic


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


class Point:
  params_name = ["CONTROL_CANCEL_GRAVITY"]

  def __init__(self):  
    self.params_value = dict()

  def read_param(self):
    self.params_value = dict()
    for p in Point.params_name:
      self.params_value[p] = float(rospy.get_param(p))

  def write_param(self):
    for p in self.params_name:
      rospy.set_param(p,self.params_value[p])

  def set_inputs(self,inputs):
    self.inputs = inputs



class LinearAC:
  def __init__(self, filename):
    rospy.init_node("LinearAC")
    self.filename = filename
    self.load()
    self.init_services_proxy()
    self.init_inputs()
    rospy.spin()


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

  def update_current_point(self,msg):
    current_inputs = self.get_inputs()

    point_update = self.interpolate_point(current_inputs)

    point_update.write_param()

    self.params_load()
    self.params_load_PID()
    return []

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

  def init_inputs(self):
    rospy.Subscriber('mavros/battery', BatteryStatus, self.cbBattery)
    self.battery_value = 12.0

  def cbBattery(self,msg):
    self.battery_value = msg.voltage

  def get_inputs(self):
    return [self.battery_value]

  def add_point(self,msg):
    point = Point()
    
    point.read_param()
    
    inputs = self.get_inputs()
    point.set_inputs(inputs)

    self.points.append(point)
    return []

  def cbSave(self,msg):
    self.save()
    return []

  def cbLoad(self,msg):
    self.load()
    return []

  def cbPlot(self,msg):
    inputs = self.get_inputs()
    x = np.linspace(msg.min,msg.max,100)
    y = np.linspace(msg.min,msg.max,100)
    
    for i in range(0,100):
      inputs[msg.input_variable] = x[i]
      pts = self.interpolate_point(inputs)
      y[i] = pts.params_value[msg.plot_variable]

    utils.plot(copy.deepcopy(x),copy.deepcopy(y))


    return []

  def save(self):
    with open(self.filename, 'w') as output:
      utils.loginfo("Save")
      outxml = gnosis.xml.pickle.dumps(self.points)
      output.write(outxml)

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


  def newfile(self):
      utils.loginfo("File \"%s\" does not exist!"%(self.filename))
      self.points = []


if __name__ == '__main__':
  LinearAC(sys.argv[1])


# Test program for the inter_extrapolate function
# import numpy as np
# from scipy.interpolate import Rbf
# from scipy.interpolate import griddata
# from scipy.interpolate import NearestNDInterpolator

# def inter_extrapolate(inputs_grid,values,inputs):
#   val = griddata(inputs_grid,values,inputs, method="linear")
#   if np.isnan(val):
#     if len(inputs)>=2:
#       nnd = NearestNDInterpolator(inputs_grid,values)
#       val = nnd(inputs)
#     else:
#       imax = np.argmax(inputs_grid)
#       imin = np.argmin(inputs_grid)
#       if np.abs(inputs_grid[imax]-inputs) < np.abs(inputs_grid[imin]-inputs):
#         val = values[imax]
#       else:
#         val = values[imin]
#   return val

# def func(x, y):
#     return x*(1-x)*np.cos(4*np.pi*x) * np.sin(4*np.pi*y**2)**2

# if __name__ == '__main__':

#   import matplotlib.pyplot as plt
#   grid_x, grid_y = np.mgrid[0:1:100j, 0:1:100j]
#   points = np.random.rand(100, 2)
#   values = func(points[:,0], points[:,1])
  

#   grid_z0 = griddata(points, values, (grid_x, grid_y), method='linear')

#   print len(grid_x)
#   print len(grid_x[0])
#   for i in range(0,len(grid_x)):
#     for j in range(0,len(grid_x[0])):
#       grid_z0[i][j] = inter_extrapolate(points, values, (grid_x[i][j], grid_y[i][j]))

#   plt.imshow(grid_z0.T, extent=(0,1,0,1), origin='lower')
#   plt.plot(points[:,0], points[:,1], 'k.', ms=2)
#   plt.show()

#EOF
