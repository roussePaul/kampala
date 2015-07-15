#!/usr/bin/env python

import rospy
from autotuner import Autotuner
from std_srvs.srv import Empty

import math

class Identification:
	method_list = {"areas":["Ziegler & Nichols"],"relay":["Internal Model Control","Ziegler & Nichols","Kappa-Tau"]}

	def __init__(self,PID,method="areas"):
		self.method = method

		# STEP
		# attributs
		self.s_initial_time = 0.0
		self.s_initial_input = 0.0

		# parameters
		self.s_param = {"amplitude":1.0, }
	def step(self,time,input):
		if self.initial_time==0.0:
			self.initial_time = time 
			self.initial_input = input

		# try to detect if the signal is stationary
