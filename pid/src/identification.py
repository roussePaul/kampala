#!/usr/bin/env python

import rospy
from autotuner import Autotuner
from std_srvs.srv import Empty

from mavros.msg import OverrideRCIn

import math

import control
from system import System
from scipy import stats
import matplotlib.pyplot as plt
import numpy as np

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

class Buffer:
	def __init__(self,N):
		self.N = N
		self.data = []

	def append(self,d):
		self.data.append(d)
		while len(self.data)>self.N:
			self.data.pop(0)
class TimeSequence:
	def __init__(self,T):
		self.T = T
		self.data = []
		self.time = []

	def append(self,s,t):
		self.data.append(s)
		self.time.append(t)
		while t-self.time[0]>self.T:
			self.data.pop(0)
			self.time.pop(0)

class Identification:
	method_list = {"relay":["Ziegler & Nichols (CL)"],
	"areas":["Internal Model Control","Ziegler & Nichols (OL)","Kappa-Tau"],
	"ramp":["Hovering"],
	"ramp_relay":["Ziegler & Nichols (CL)"],
	"stairs":["Stairs"]}

	def __init__(self,method="areas"):
		self.method = method

		# STEP
		# attributs
		self.s_initial_time = 0.0
		self.s_initial_input = 0.0

		# parameters
		self.s_param = {"amplitude":1.0, "step_size":2.0}
		self.r_param = {"amplitude":0.1, "oscillations":4, "hysteresys":0.1}
		self.ramp_param = {"rate":0.1,"command_start":0.2}

		# state if the identification process
		# states: "wait", "initialize" "in progress", "identify", "done"
		self.state = "wait"
		self.U_seq = TimeSequence(100)
		self.Y_seq = TimeSequence(100)
		self.state = "online"


	def start(self):
		self.state = "initialize"

	def online(self, u, ym, t):
		self.U_seq.append(u,t)
		self.Y_seq.append(ym,t)

		s = control.tf([1.0,0.0],[1.0])
		int_U = System( 1.0/s**2 )

		i2u = [0.0]
		i2u_0 = [0.0]
		
		for i in range(1,len(self.U_seq.time)):
			i2u.append( int_U.output(self.U_seq.data[i],self.U_seq.time[i]) )
			int_U.next_state()
			i2u_0.append( (self.U_seq.time[i]-self.U_seq.time[0])**2/2.0 )

		A = np.vstack([i2u,i2u_0]).T
		a, b = np.linalg.lstsq(A, self.Y_seq.data)[0]
		K = a
		u_0 = -b/K

		self.identification = {"K":K,"u0":u_0}
		print self.identification


	def get_command(self, input, time):
		#print self.method
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
			self.Rv = Relay(1)
			self.T = time
			self.U = self.ramp_param["command_start"]
			self.T0 = time
			self.state = "ramp"

			s = control.tf([1.0,0.0],[1.0])
			wc = 1.0*2.0*math.pi
			d = s/(1+s/wc)
			self.acc = System( d**2 )
			self.vel = System( d )
			self.int = System( 1.0/s )

			self.datax = []
			self.datay = []
			return 0.0

		if self.state == "ramp":

			acc = self.acc.output(input,time)
			self.acc.next_state()
			vel = self.vel.output(input,time)
			self.vel.next_state()

			(s,switch) = self.R.output(input)
			(v,sw) = self.Rv.output(0.05+vel)

			self.U = self.ramp_param["command_start"] + self.int.output(s,time)
			self.int.next_state()


			print self.U

			if math.fabs(vel) >= 0.1:
				self.datax = np.append(self.datax,input)
				self.datay = np.append(self.datay,self.U)


			if switch and time-self.T0>1.0:
				print self.datax
				print self.datay
				fig1 = plt.figure()
				ax1 = fig1.add_subplot(111)
				ax1.plot(self.datax, 'ro')
				plt.show()
				slope, intercept, r_value, p_value, std_err = stats.linregress(self.datax,self.datay)

				# K = 1.0/slope
				# T0

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


	def stairs(self,input,time):

		# print self.state

		if self.state == "wait":
			return 0.0

		if self.state == "initialize":
			self.datax = []
			self.datay = []
			self.takeoff = False
			self.time = time
			self.state = "takeoff"

			s = control.tf([1.0,0.0],[1.0])
			wc = 5.0*2.0*math.pi
			d = s/(1+s/wc)
			self.vel = System( d )
			self.acc = System( d**2 )
			return 0.0

		if self.state == "takeoff":

			i = math.floor(time-self.time)
			self.U = self.ramp_param["command_start"] + 0.02*i


			vel = self.vel.output(input,time)
			self.vel.next_state()

			acc = self.vel.output(input,time)
			self.vel.next_state()

			# print self.U
			# print "U {0:+.4f}  vel {1:+.4f}  acc {2:+.4f}".format(self.U,vel,acc)

			if math.fabs(vel)>=0.05 and time-self.time>1.0:
				self.u0 = self.U-0.1
				self.datax = []
				self.datay = []
				self.time = time
				self.o = 5
				self.accX = []
				self.accY = []
				self.state = "opt2"
				self.R = Relay(1)
				self.R.output(input)

			return self.U

		if self.state == "opt2":

			if self.o==0:
				self.state = "identify1"
				return self.U

			self.datax = np.append(self.datax,time)
			self.datay = np.append(self.datay,input)

			vel = self.vel.output(input,time)
			self.vel.next_state()

			(v,s) = self.R.output(input)

			if self.time-time>1.0 or s:
				print self.datay
				print self.datax
				coeffs = np.polyfit(self.datax, self.datay, 2)
				self.accX.append(-coeffs[0])
				self.accY.append(self.U)
				print self.accX
				self.o -= 1
				self.datax = []
				self.datay = []
				self.time = time


				if len(self.accX)>=2:
					c = self.accX
					u = self.accY

					vel = self.vel.output(input,time)
					self.vel.next_state()

					p = np.polyfit(c, u, 1)
					self.u0 = float(p[1])
					self.K = float(2.0/p[0])

					print "--------------------------------------"
					print self.K
					print self.u0
					print "--------------------------------------"
					dt = 3.0
					self.U = self.u0 + v*math.fabs(vel*2.0/(self.K*dt))

					print "U "+ str(self.U)
					print "V "+ str(v)
				else:
					self.U = self.u0 + v*0.3




			return self.U

		if self.state == "optimize":

			if self.o==0:
				self.state = "identify1"
				return self.U

			self.datax = np.append(self.datax,time)
			self.datay = np.append(self.datay,input)

			vel = self.vel.output(input,time)
			self.vel.next_state()


			if self.time-time>3.0 or input<0:
				print self.datay
				print self.datax
				coeffs = np.polyfit(self.datax, self.datay, 2)
				self.accX.append(-coeffs[0])
				self.accY.append(self.U)
				print self.accX
				self.U += 0.2
				self.o -= 1
				self.datax = []
				self.datay = []
				self.time = time

			return self.U
		if self.state == "identify1":

			c = self.accX
			u = self.accY

			vel = self.vel.output(input,time)
			self.vel.next_state()


			p = np.polyfit(c, u, 1)
			self.u0 = float(p[1])
			self.K = float(2.0/p[0])

			print "--------------------------------------"
			print self.K
			print self.u0
			print "--------------------------------------"


			self.n_osc = 0
			self.time = time
			self.R = Relay(1)
			self.state = "osc1"
			return self.U

		if self.state == "osc1":
			vel = self.vel.output(input,time)
			self.vel.next_state()


			self.datax = np.append(self.datax,time)
			self.datay = np.append(self.datay,input)

			(v,s) = self.R.output(input)

			print s

			if s:
				dt = 3.0
				self.time = time + dt 
				self.U = self.u0 - vel/(self.K*dt)

				if self.n_osc>0:
					coeffs = np.polyfit(self.datax, self.datay, 2)
					self.accX.append(-coeffs[0])
					self.accY.append(self.U)
					self.datax = []
					self.datay = []

				self.n_osc += 1

				if self.n_osc>3:
					self.state = "identify2"

			return self.U

		if self.state == "identify2":

			c = self.accX
			u = self.accY

			vel = self.vel.output(input,time)
			self.vel.next_state()


			p = np.polyfit(c, u, 1)
			self.u0 = float(p[1])
			self.K = float(2.0/p[0])

			print "--------------------------------------"
			print self.K
			print self.u0
			print "--------------------------------------"


			fig1 = plt.figure()
			ax1 = fig1.add_subplot(111)
			ax1.plot(c,u, 'ro')
			plt.show()


			self.n_osc = 0
			self.time = time
			self.state = "done"
			return self.U