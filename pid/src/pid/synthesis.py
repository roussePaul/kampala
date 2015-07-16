#!/usr/bin/env python

import rospy
from autotuner import Autotuner
from std_srvs.srv import Empty

import math

class Synthesis:
	method_list = {"relay":["Ziegler & Nichols (CL)"],"areas":["Internal Model Control","Ziegler & Nichols (OL)","Kappa-Tau"]}

	def __init__(self,method):
		self.method = method

	def synthetise(self,params):
		if self.method[0] == ("relay"):
			if self.method[1] == "Ziegler & Nichols (CL)":
				return {"K":0.6*params["Ku"],"Ti":0.5*params["Tu"],"Td":0.125*params["Tu"],"N":1000.0,"b":1.0,"c":1.0}