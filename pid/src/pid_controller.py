#!/usr/bin/env python

import rospy
from autotuner import Autotuner
from identification import Identification
from synthesis import Synthesis
from std_srvs.srv import Empty

from pid.srv import GetPIDParameters, SetPIDParameters, Autotune, Mode

import numpy as np
import math

import analysis
import utils

import control
from system import System

class PID:
    parameters = {"K":0.3,"Ti":10000.0,"Td":0.0,"b":1.0,"c":1.0,"N":100.0,"u0":0.3,"I_lim":10000.0}
    current_pid_id = 0
    mode_list = ["controller","online identification","identification"]

    def __init__(self, name=""):
        # init name
        self.name = name
        if self.name=="":
            self.name = "PID_"+str(PID.current_pid_id)
            PID.current_pid_id += 1
        self.path = rospy.get_name() +"/"+ self.name + "/" 
        utils.logerr(self.path)
        # init node
        Autotuner.add_controller(self.name)

        self.read_params()
        self.write_params()
        self.init_controller()
        self.init_services()

        # mode of the controller: "controller", "identification"
        self.mode = ["controller"]

    def init_services(self):
        rospy.Service(self.path+'set_mode', Mode, self.set_mode)
        rospy.Service(self.path+'set_gains', SetPIDParameters, self.c_set_params)
        rospy.Service(self.path+'get_gains', GetPIDParameters, self.c_get_params)


    def controller(self,ym,y0,time):
        if self.mode == "identification":
            if self.identifier.state !="done":
                u = self.identifier.get_command(y0-ym,time)
            else:
                params = self.identifier.identification
                print params
                gains = self.synthesiser.synthetise(params)
                self.set_params(gains)
                self.mode = "controller"
                u = self.get_command(y0-ym,time)

        if "controller" in self.mode:
            u = self.get_command(y0-ym,time)

        if "online identification" in self.mode:
            self.identifier.online(u,ym,time)

        return u

    def set_mode(self,msg):
        if not(msg.mode in PID.mode_list):
            utils.logwarn("PID mode does not exist!")

        if msg.run:
            if not(msg.mode in self.mode):
                self.mode.append(msg.mode)
        else:
            if msg.mode in self.mode:
                self.mode.remove(msg.mode)

        if "online identification" in self.mode:
            self.identifier = Identification()


        utils.loginfo("Modes : " + str(self.mode))
        return {'success':True}

# define updates of the parameters of the pid
    def read_params(self):
        for (p,v) in PID.parameters.iteritems():
            setattr(self, p, rospy.get_param(self.name+"/"+p,v))

    def write_params(self):
        for (p,v) in PID.parameters.iteritems():
            rospy.set_param(self.name+"/"+p,getattr(self, p))


    def c_set_params(self,msg):
        params = dict(zip(msg.keys,msg.values))
        self.set_params(params)
        return []

    def set_params(self,params):
        print params
        for param_name in params:
            if hasattr(self,param_name):
                setattr(self,param_name,params[param_name])
                print param_name +  " " +str(getattr(self,param_name))
        self.write_params()

        self.init_controller()

    def c_get_params(self,msg):
        srv = Controller
        params = {"N":self.N,"Ti":self.Ti,"Td":self.Td,"b":self.b,"c":self.c,"K":self.K,"I_lim":self.I_lim,"u0":self.u0}
        srv.keys = params.keys()
        srv.values = a.values()
        return  msg

    # PID controller
    def init_controller(self):
        Ti = self.Ti
        Td = self.Td
        K = self.K
        N = self.N

        s = control.tf([1.0,0.0],[1.0])

        # PID filter
        self.I = System( K/(Ti*s) )
        self.PD = System( K*(1.0 + s*Td/(1.0+Td*s/N)) )

    def get_command(self,e,time):
        u_pd = self.PD.output(e,time)
        u_i = self.I.output(e,time)
        self.PD.next_state()

        # Freezing of the integral action if too large
        if math.fabs(u_i) < self.I_lim:
            self.I.next_state()
        else:
            self.I.freeze_state()

        output = u_pd+u_i

        return output + self.u0

if __name__=="__main__":
    rospy.init_node("pid_test")
    pid = PID("test_1")
    PID()
    PID()
    PID()

    pid.read_params()
    for i in range(1,100):
        pid.get_command(math.sin(i*0.1),i*0.1)
    rospy.spin()

#EOF