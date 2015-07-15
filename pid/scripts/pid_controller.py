#!/usr/bin/env python

import rospy
from autotuner import Autotuner
from std_srvs.srv import Empty

from pid.srv import GetPIDParameters, SetPIDParameters, Autotune

import numpy as np
from numpy.linalg import inv
import math

class PID:
    current_pid_id = 0
    def __init__(self, name=""):
        # init name
        self.name = name
        if self.name=="":
            self.name = "PID_"+str(PID.current_pid_id)
            PID.current_pid_id += 1
        self.path = self.name + "/" 
        # init node
        Autotuner.add_controller(self.name)

        self.read_params()
        self.init_controller()
        self.init_services()

        # mode of the controller: "controller", "identification"
        self.mode = "controller"
    def init_services(self):
        rospy.Service(self.path+'autotune', Autotune, self.identify)
        rospy.Service(self.path+'set_gains', SetPIDParameters, self.set_params)
        rospy.Service(self.path+'get_gains', GetPIDParameters, self.get_params)


    def controller(self,y0,ym,time):
        if self.mode == "controller":
            return self.get_command(y0,ym,time)

        if self.mode == "identifier":
            if self.identifier.state !="done":
                self.identifier.get_command(y0-ym,time)
            else:
                params = self.identifier.params
                gains = self.synthesiser.synthetise(params)
                self.K = gains["K"]
                self.Ti = gains["Ti"]
                self.Td = gains["Td"]
                self.b = gains["b"]
                self.c = gains["c"]
                self.N = gains["N"]
                self.write_params()
                self.mode = "controller"



    def identify(self,msg):
        self.identifier = Identification(msg.method1)
        self.synthesiser = Synthesis([msg.method1,msg.method2])
        self.mode = "identification"
        return []

# define updates of the parameters of the pid
    def read_params(self):
        self.K = rospy.get_param(self.name+"/K",1.0)
        self.Ti = rospy.get_param(self.name+"/Ti",10000.0)
        self.Td = rospy.get_param(self.name+"/Td",0.001)
        self.b = rospy.get_param(self.name+"/b",1.0)
        self.c = rospy.get_param(self.name+"/c",1.0)
        self.N = rospy.get_param(self.name+"/N",100.0)

    def write_params(self):
        rospy.set_param(self.name+"/K",self.K)
        rospy.set_param(self.name+"/Ti",self.Ti)
        rospy.set_param(self.name+"/Td",self.Td)
        rospy.set_param(self.name+"/b",self.b)
        rospy.set_param(self.name+"/c",self.c)
        rospy.set_param(self.name+"/N",self.N)

    def set_params(self,msg):
        params = dict(zip(msg.keys,msg.values))
        for param_name in msg.params:
            if hasattr(self,param_name):
                self[param_name] = msg.params[param_name]
        self.write_params()

        return []

    def get_params(self,msg):
        srv = Controller
        params = {"N":self.N,"Ti":self.Ti,"Td":self.Td,"b":self.b,"c":self.c,"K":self.K}
        srv.keys = params.keys()
        srv.values = a.values()
        return  msg

    # PID controller
    def init_controller(self):
        # state vector
        self.X = np.matrix([0.0]*2).T
        self.y = np.matrix([0.0]*2).T
        self.time = 0.0

        Ti = self.Ti
        Td = self.Td
        b = self.b
        c = self.c
        K = self.K
        N = self.N

        # Create the state space representation of the filter
        d = np.matrix([Ti*Td/N, Ti, 0])
        a0 = np.matrix([K*b*Ti*Td/N+c*Td*Ti, K*b*Ti+Td/N, 1])
        am = np.matrix([-K*Ti*Td/N-Td*Ti, -K*Ti-Td/N, -1])


        (s, M) = d.shape

        d_n = d/d.item((0,0))
        a0_n = a0/d.item((0,0))
        am_n = am/d.item((0,0))

        c0_n = a0_n - a0_n.item((0,0))*d_n
        cm_n = am_n - am_n.item((0,0))*d_n

        d_r = np.matrix( d_n.getA()[0,1:] )
        c0_r = np.matrix( c0_n.getA()[0,1:] )
        cm_r = np.matrix( cm_n.getA()[0,1:] )

        In = np.concatenate((np.identity(M-2), np.zeros((1, M-2))), axis=0)

        self.A = np.concatenate((-d_r.T,In),axis=1)
        self.B = np.concatenate((c0_r.T,cm_r.T),axis=1)
        self.C = np.concatenate((np.matrix([[1]]), np.zeros((1, M-2))), axis=1)
        self.D = np.matrix( [a0_n.item((0,0)), am_n.item((0,0))] )
        self.M = M


    def get_command(self,ym,y0,time):
        # Update vectors of the state representation
        A = self.A
        B = self.B
        C = self.C
        D = self.D
        M = self.M

        # Get time difference
        dt = time-self.time
        self.time = time

        I = np.identity(M-1)
        Ai = inv(I-dt/2.0*A)
        phi = Ai*(I+dt/2.0*A)
        gamma = Ai*dt*B

        # Apply the filter
        y = np.matrix([[y0],[ym]])
        self.X = phi*self.X + gamma*1/2.0*(y+self.y)
        self.y = y

        U = C*self.X + D*y

        return U.item((0,0))

if __name__=="__main__":
    rospy.init_node("pid_test")
    pid = PID("test_1")
    PID()
    PID()
    PID()

    pid.read_params()
    for i in range(1,100):
        pid.get_command(math.sin(i*0.1),math.cos(i*0.1),i*0.1)
    rospy.spin()

#EOF