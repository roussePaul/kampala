#!/usr/bin/env python

import rospy
from autotuner import Autotuner
from std_srvs.srv import Empty

from pid.srv import GetPIDParameters, SetPIDParameters

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

        self.init_state()
        self.init_services()

    def init_services(self):
        rospy.Service(self.path+'identify', Empty, self.identify)
        rospy.Service(self.path+'set_gains', SetPIDParameters, self.set_params)
        rospy.Service(self.path+'get_gains', GetPIDParameters, self.get_params)

    def init_state(self):
        # command (output) vector
        self.U = [0.0]*3

        # input measure
        self.Ym = [0.0]*3

        # input setpoint
        self.Y0 = [0.0]*3

        # Time vector
        self.T = [0.0]*2

    def identify(self):
        pass

# define updates of the parameters of the pid
    def read_params(self):
        self.K = rospy.get_params(self.name+"/K",1.0)
        self.Ti = rospy.get_params(self.name+"/Ti",0.0)
        self.Td = rospy.get_params(self.name+"/Td",0.0)
        self.b = rospy.get_params(self.name+"/b",1.0)
        self.c = rospy.get_params(self.name+"/c",1.0)
        self.N = rospy.get_params(self.name+"/N",100.0)

    def write_params(self):
        rospy.set_params(self.name+"/K",self.K)
        rospy.set_params(self.name+"/Ti",self.Ti)
        rospy.set_params(self.name+"/Td",self.Td)
        rospy.set_params(self.name+"/b",self.b)
        rospy.set_params(self.name+"/c",self.c)
        rospy.set_params(self.name+"/N",self.N)

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

# give the output of the PID controller
    def get_command(self,ym,y0,time):
        # Update  Ym, Y0 and time
        self.Ym.pop(0)
        self.Ym.append(ym)

        self.Y0.pop(0)
        self.Y0.append(y0)

        self.T[0] = self.T[1]
        self.T[1] = time

        # Update vectors of the state representation
        N = self.N
        Ti = self.Ti
        Td = self.Td
        b = self.b
        c = self.c
        K = self.K

        # Get time difference
        Te = self.T[1]-self.T[0]

        # apply PID filter
        n = [ 4*N + 2*Td*Te + 2*K*N*Te*Ti*b + K*Td*Te^2*Ti*b + N*Td*Te^2*Ti*c, 2*K*Td*Te^2*Ti*b - 8*N + 2*N*Td*Te^2*Ti*c, 4*N - 2*Td*Te - 2*K*N*Te*Ti*b + K*Td*Te^2*Ti*b + N*Td*Te^2*Ti*c]
        num_value_0 = np.dot(n, self.Y0)

        n = [ 4*N + 2*Td*Te + 2*K*N*Te*Ti + K*Td*Te^2*Ti + N*Td*Te^2*Ti, 2*K*Td*Te^2*Ti - 8*N + 2*N*Td*Te^2*Ti, 4*N - 2*Td*Te - 2*K*N*Te*Ti + K*Td*Te^2*Ti + N*Td*Te^2*Ti] 
        num_value_m = np.dot(n, self.Ym)

        d = [ Te*(2*N + Td*Te*Ti), Te*(2*N + Td*Te*Ti) - Te*(2*N - Td*Te*Ti), -Te*(2*N - Td*Te*Ti)]
        den_value = np.dot(d, self.U)
        
        n_u = (num_value_0 - num_value_m)/den_value
        n_u = n_u.item()

        # Update U
        self.U.pop(0)
        self.U.append(n_u)



if __name__=="__main__":
    rospy.init_node("pid_test")
    PID("test_1")
    PID()
    PID()
    PID()
    rospy.spin()

#EOF