#!/usr/bin/env python

import rospy
from autotuner import Autotuner
from std_srvs.srv import Empty

from mavros.msg import OverrideRCIn

import math

import control
from system import System

class Relay:
	def __init__(self,D,E=0.0):
		self.D = D
		self.E = E
		self.U = D

	def output(self,I):
		if I>=self.E:
			U = self.D
		elif I<-self.E:
			U = -self.D
		else:
			U = self.U

		switch = (self.U!=U)

		self.U = U

		return (self.U,switch)

class Identification:
	method_list = {"relay":["Ziegler & Nichols (CL)"],
	"areas":["Internal Model Control","Ziegler & Nichols (OL)","Kappa-Tau"],
	"ramp":["Hovering"],
	"ramp_relay":["Ziegler & Nichols (CL)"]}

	def __init__(self,method="areas"):
		self.method = method

		# STEP
		# attributs
		self.s_initial_time = 0.0
		self.s_initial_input = 0.0

		# parameters
		self.s_param = {"amplitude":1.0, "step_size":2.0}
		self.r_param = {"amplitude":0.1, "oscillations":4, "hysteresys":0.1}
		self.ramp_param = {"rate":0.01,"command_start":0.0}

		# state if the identification process
		# states: "wait", "initialize" "in progress", "identify", "done"
		self.state = "wait"

	def start(self):
		self.state = "initialize"

	def get_command(self, input, time):
		print self.method
		methodToCall = getattr(self, self.method)
		if not methodToCall:
			raise Exception("Method %s not implemented" % method_name)
		return methodToCall(input,time)

	def areas(self, input, time):
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
			return 0.0

	def relay(self,input,time):

		print self.state

		if self.state == "wait":
			return 0.0

		if self.state == "initialize":
			self.n_switch = 0
			self.T_switch = []
			self.R = Relay(self.r_param["amplitude"],self.r_param["hysteresys"])
			self.initial_time = time 
			self.y = []
			self.time = []
			self.state = "in progress"
			return 0.0

		if self.state == "in progress":

			self.y.append(input)
			self.time.append(time-self.initial_time)

			(U,switch) = self.R.output(input)

			if switch:
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

	def ramp(self,input,time):

		print self.state

		if self.state == "wait":
			return 0.0

		if self.state == "initialize":
			self.R = Relay(self.ramp_param["rate"])
			self.T = time
			self.U = self.ramp_param["command_start"]
			self.T0 = time
			self.state = "in progress"
			return 0.0

		if self.state == "in progress":

			(s,switch) = self.R.output(input)

			self.U += (time-self.T)*s
			
			self.T = time

			if switch and time-self.T0>1.0:
				self.state = "identify"

			return self.U

		if self.state == "identify":
			self.identification = {"u0":self.U}

			self.state = "done"
			return 0.0


	def ramp_relay(self,input,time):

		print 
		print self.state

		if self.state == "wait":
			return 0.0

		if self.state == "initialize":
			self.R = Relay(self.ramp_param["rate"])
			self.Rv = Relay(0.02)
			self.T = time
			self.U = self.ramp_param["command_start"]
			self.T0 = time
			self.state = "ramp"

			s = control.tf([1.0,0.0],[1.0])
			wc = 30.0*2.0*math.pi
			d = s/(1+s/wc)
			self.acc = System( d**2 )
			self.vel = System( d )

			return 0.0

		if self.state == "ramp":

			v = self.vel.output(input,time)
			self.vel.next_state()
			a = self.acc.output(input,time)
			self.acc.next_state()

			acc = 1.0
			if math.fabs(v)>0.02:
				acc = -1.0

			(s,switch) = self.R.output(input)

			self.U = s*acc

			if switch and time-self.T0>1.0:
				self.n_switch = 0
				self.T_switch = []
				self.R = Relay(self.r_param["amplitude"],self.r_param["hysteresys"])
				self.initial_time = time 
				self.y = []
				self.time = []
				self.state = "relay"
				self.u0 = self.U


			return self.U

		if self.state == "relay":
			self.y.append(input)
			self.time.append(time-self.initial_time)

			(U,switch) = self.R.output(input)

			if switch:
				self.n_switch += 1
				self.T_switch.append(time)
			
			if self.n_switch>self.r_param["oscillations"]:
				self.state = "identify"

			print U
			print input

			return U+self.u0

		if self.state == "identify":

			Tu = self.T_switch[-1]-self.T_switch[-2]

			A = (max(self.y) - min(self.y)) / 2.0
			D = self.r_param["amplitude"]

			Ku = 4*D/(math.pi*A)

			self.identification = {"Ku":Ku,"Tu":Tu,"u0":self.u0}

			self.state = "done"
			return 0.0