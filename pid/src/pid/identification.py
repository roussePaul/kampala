#!/usr/bin/env python

import rospy
from autotuner import Autotuner
from std_srvs.srv import Empty

import math

class Identification:
	method_list = {"relay":["Ziegler & Nichols (CL)"],"areas":["Internal Model Control","Ziegler & Nichols (OL)","Kappa-Tau"]}

	def __init__(self,method="areas"):
		self.method = method

		# STEP
		# attributs
		self.s_initial_time = 0.0
		self.s_initial_input = 0.0

		# parameters
		self.s_param = {"amplitude":1.0, "step_size":2.0}

		self.r_param = {"amplitude":0.5, "oscillations":4, "hysteresys":0.1}

		# state if the identification process
		# states: "wait", "initialize" "in progress", "identify", "done"
		self.state = "wait"

	def start(self):
		self.state = "initialize"

	def get_command(self, input, time):
		print self.method
		if self.method == "areas":
			return self.step(input,time)
		if self.method == "relay":
			return self.relay(input,time)

	def step(self, input, time):
		if self.state == "initialize":
			self.initial_time = time 
			self.y = []
			self.time = []
			self.state = "in progress"
			return 0


		if self.state == "in progress":
			self.y.append(input)
			self.time.append(time-self.initial_time)

			if time-self.initial_time>self.s_param["step_size"]:
				self.state = "identify"
				return 0
			else :	
				As = self.s_param["amplitude"]
				return As

		if self.state == "identify":
			y_0 = self.y[0]
			mu = self.y[-1]

			# Get A0
			time = 0.0
			A0 = 0.0
			As = self.s_param["amplitude"]
			mu_s = mu/As
			for (t,y) in zip(self.time,self.y):
				dt = t-time
				time = t
				y_us = y/As
				A0 += dt*(mu-y_us)

			t0 = A0/mu_s

			time = 0.0
			A1 = 0.0
			for (t,y) in zip(self.time,self.y):
				if t>t0:
					continue
				dt = t-time
				time = t
				A1 += dt*y_us

			T = math.exp(1.0)*A1/mu_s
			L = (A0-math.exp(1.0)*A1)/mu_s

			
			self.identification = {"mu":mu_s,"T":T,"L":L}

			self.state = "done"
			return 0

	def relay(self,input,time):

		print self.state

		if self.state == "wait":
			return 0.0

		if self.state == "initialize":
			self.n_switch = 0
			self.T_switch = []
			self.current_U = self.r_param["amplitude"]
			self.initial_time = time 
			self.y = []
			self.time = []
			self.state = "in progress"
			return 0.0

		if self.state == "in progress":


			E = self.r_param["hysteresys"]
			D = self.r_param["amplitude"]

			self.y.append(input)
			self.time.append(time-self.initial_time)

			if input>E:
				U = D
			elif input<-E:
				U = -D
			else:
				U = self.current_U

			if U != self.current_U:
				self.n_switch += 1
				self.T_switch.append(time)
			
			self.current_U = U

			if self.n_switch>self.r_param["oscillations"]:
				self.state = "identify"

			print U
			print input

			return U

		if self.state == "identify":
			Tu = self.T_switch[-1]-self.T_switch[-2]

			A = (max(self.y) - min(self.y)) / 2.0
			D = self.r_param["amplitude"]

			Ku = 4*D/(math.pi*A)

			self.identification = {"Ku":Ku,"Tu":Tu}

			self.state = "done"
			return 0.0