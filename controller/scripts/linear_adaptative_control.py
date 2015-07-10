#!/usr/bin/env python

# PID controller for the IRIS+ in the SML Lab 
# Gets inputs from the Security Guard and the Trajectory Generator
# Publishes commands via Mavros' rc/override topic


import rospy
import sys
import math

from mavros.msg import BatteryStatus

import analysis
import utils

from scipy.interpolate import griddata

import pickle

class Point:
  params_name = ["CONTROL_CANCEL_GRAVITY","CONTROL_ARMING_MIN","N_yaw","K_yaw","w_inf","Ktt","Kphi","PID_w","PID_w_z","PID_I_lim","PID_K_i"]

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
    self.filename = filename

  def init_services_proxy(self):
    try: 
      self.params_load = rospy.ServiceProxy("/%s/blender/update_parameters"%(self.name), Empty)
      self.params_load_PID = rospy.ServiceProxy("/%s/PID_controller/update_parameters"%(self.name), Empty)
    except rospy.ServiceException as exc:
      utils.loginfo("PID not reachable " + str(exc))

  def update_current_point(self):
    current_inputs = self.get_inputs()

    point_update = self.interpolate_point(current_inputs)

    point_update.write_param()

  def interpolate_point(self, current_inputs):
    pts = Point()
    
    inputs_grid = [p.inputs for p in self.points]
    
    new_param = dict()

    for p_name in Point.params_name:
      values = [p.params_value[p_name] for p in self.points]
      new_param[p_name] = griddata(inputs, values, current_inputs, method='linear')

    pts.params_value = new_param

    pts.set_inputs(current_inputs)

    return pts

  def init_inputs(self):
    rospy.Subscriber('mavros/battery', BatteryStatus, self.cbBattery)

  def cbBattery(self,msg):
    self.battery_value = msg.voltage

  def get_inputs(self):
    return [self.battery_value]

  def add_point(self):
    point = Point()
    
    point.load_from_param()
    
    inputs = self.get_inputs()
    point.set_inputs(inputs)

    self.points.append(point)

  def save(self):
    with open(self.filename, 'wb') as output:
      pickle.dump(self, output, pickle.HIGHEST_PROTOCOL)

  def load(self):
    if self.filename == "":
      utils.loginfo("Linear Adaptative Controller not initialised, starting with the actual controller")
      self.points = []
    else:
      utils.loginfo("Loading file: "+self.filename)
      with open(self.filename, 'rb') as input:
        self = pickle.load(input)




#EOF