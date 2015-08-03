#!/usr/bin/env python

import rospy
from autotuner import Autotuner
from std_srvs.srv import Empty

import math

class Synthesis:

	def __init__(self,method):
		self.method = method

	def synthetise(self,params):
		if self.method[0] == ("relay"):
			if self.method[1] == "Ziegler & Nichols (CL)":
				return {"K":0.6*params["Ku"],"Ti":0.5*params["Tu"],"Td":0.125*params["Tu"],"N":1000.0,"b":1.0,"c":1.0}

		if self.method[0] == ("ramp"):
			if self.method[1] == "Hovering":
				return {"u0":params["u0"]}

		if self.method[0] == ("ramp_relay"):
			if self.method[1] == "Ziegler & Nichols (CL)":
				return {"K":0.6*params["Ku"],"Ti":0.5*params["Tu"],"Td":0.125*params["Tu"],"N":1000.0,"b":1.0,"c":1.0,"u0":params["u0"]}

		if self.method[0] == ("stairs"):
			if self.method[1] == "Stairs":
				return {"u0":params["u0"]}

		if self.method[0] == ("throttle"):
			print params
			k = 0.3

			w1 = 1.0
			w0 = 1.0
			xi = 0.7

			a1 = 1.0/(w1*w0**2)
			a2 = 2.0*xi/(w1*w0) + 1.0/w0**2
			a3 = 1.0/w1 + 2.0*xi/w0

			K = a3/(a1*k)
			Td = a2/a3
			Ti = a3

			return {"K":K,"Ti":Ti,"Td":Td,"N":100.0,"b":1.0,"c":1.0,"u0":params["u0"]}

		return dict()