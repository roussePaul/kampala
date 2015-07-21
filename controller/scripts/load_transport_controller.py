#!/usr/bin/python

import rospy
from mocap.msg import QuadPositionDerived, QuadPositionDerivedExt


from controller_base import Controller
import numpy
from numpy import *
from numpy import cos as c
from numpy import sin as s

#Constants
#*************************************
NODE_NAME='LoadTransportController'
#*************************************


class LoadTransportController(Controller):
    def __init__(self):
        self.load_parameters()
        #rospy.Service('PID_controller/update_parameters', Empty, self.update_parameters)


    # Read parameters for controller
    def load_parameters(self):    
        self.parameters = ParametersSys();  


    # target_point is for the load and should be QuadPositionDerivedExt!
    def  get_output(self,current_point_load,current_point_vehicle,target_point):
        parameters = self.parameters

        # transported mass (kg)
        M  = parameters.M
        
        # mass of vehicles (kg)
        m = parameters.m
        
        # acceleration due to gravity (m/s^2)
        g  = parameters.g
        
        # cable length (m)
        L = parameters.L
        
        # third canonical basis vector
        e3 = numpy.array([0.0,0.0,1.0])
        
        #--------------------------------------#
        # The coordinates needs to be rotated around the x-axis

        # transported mass: position and velocity
        x,x_vel,x_acc=get_pos_vel_acc(current_point_load)
        xM = numpy.array([x[0], -x[1], -x[2]])
        vM = numpy.array([x_vel[0], -x_vel[1], -x_vel[2]])
        # vehicle: position and velocity
        x,x_vel,x_acc=get_pos_vel_acc(current_point_vehicle)
        x = numpy.array([x[0], -x[1], -x[2]])
        v = numpy.array([x_vel[0], -x_vel[1], -x_vel[2]])
        
        #--------------------------------------#
        # desired LOAD trajectory
        xd,xd_vel,xd_acc=get_pos_vel_acc(target_point)
        xd = numpy.array([xd[0], -xd[1], -xd[2]])
        vd = numpy.array([xd_vel[0], -xd_vel[1], -xd_vel[2]])
        ad = numpy.array([xd_acc[0], -xd_acc[1], -xd_acc[2]])
        jd = numpy.array([target_point.x_jerk, -target_point.y_jerk, -target_point.z_jerk])
        sd = numpy.array([target_point.x_snap, -target_point.y_snap, -target_point.z_snap])
        
        #--------------------------------------#
        # position error and velocity error
        ep = xd - xM
        ev = vd - vM
        
        #--------------------------------------#
        G     = g*e3 - ad
        Gdot  = -jd
        G2dot = -sd
        
        G_all = concatenate([G,Gdot,G2dot])
        
        #--------------------------------------#
        
        # cable direction
        n = (xM - x)/numpy.linalg.norm(xM - x);
        # cable angular velocity
        w = skew(n).dot(vM - v)/numpy.linalg.norm(xM - x)
        
        #--------------------------------------#

        TT,tau,nTd,ew = UniThurstControlComplete(ep,ev,n,w,G_all,parameters)

        U = n*(-TT*(m+M) + dot(w,w)*m*L) + OP(n).dot(-tau*m*L)

        #print numpy.linalg.norm(U)

        return numpy.array([U[0], -U[1], -U[2]])/m

#--------------------------------------------------------------------------#

def get_pos_vel_acc(obj):
    x=(obj.x,obj.y,obj.z,obj.yaw)
    x_vel=(obj.x_vel,obj.y_vel,obj.z_vel,obj.yaw_vel)
    x_acc=(obj.x_acc,obj.y_acc,obj.z_acc,obj.yaw_acc)
    return x,x_vel,x_acc

#--------------------------------------------------------------------------#
# Some functions
def Rx(tt):
    
    return numpy.array([[1.0,0.0,0.0],[0.0,c(tt),-s(tt)],[0.0,s(tt),c(tt)]])

# print Rx(60*3.14/180)

def Ry(tt):
    
    return numpy.array([[c(tt),0.0,s(tt)],[0.0,1,0.0],[-s(tt),0.0,c(tt)]])

# print Ry(60*3.14/180)

def Rz(tt):
    
    return numpy.array([[c(tt),-s(tt),0.0],[s(tt),c(tt),0.0],[0.0,0.0,1]])

# print Rz(60*3.14/180)

def skew(xx):
    
    x = xx[0]
    y = xx[1]
    z = xx[2]
    
    return numpy.array([[0,-z,y],[z,0,-x],[-y,x,0]])

# print skew([1,2,3])

#--------------------------------------------------------------------------#
# orthogonal projection operator
def OP(x):
    
    return -skew(x).dot(skew(x))

#print OP([1,2,3])
#print OP([1,0,0])

#--------------------------------------------------------------------------#
# unit vector
def unit_vec(psi,theta):

    e1  = numpy.array([1.0,0.0,0.0])
    aux = Rz(psi).dot(e1)
    aux = Ry(theta).dot(aux)

    return aux

#print unit_vec(45*3.14/180,0)
#print unit_vec(45*3.14/180,45*3.14/180)
#print unit_vec(0*3.14/180,-90*3.14/180)


#--------------------------------------------------------------------------#

def UniThurstControlComplete(ep,ev,n,w,G_all,parameters):

    nDot = skew(w).dot(n)

    u, Td, nTd, Tdnorm = UniThrustControl(ep,ev,G_all,parameters)

    error_thrust = OP(n).dot(Td)

    evDot = -u -error_thrust

    u, Td, nTd, Tdnorm, uDot, TdDot, nTdDot, TdnormDot = UniThrustControlDot(ep,ev,evDot,G_all,parameters)

    error_thrust_dot = -skew(nDot).dot(skew(n).dot(Td)) - skew(n).dot(skew(nDot).dot(Td)) - skew(n).dot(skew(n).dot(TdDot))

    ev2Dot = -uDot -error_thrust_dot

    tau, ew = UniThrustControlAngVel(ep,ev,evDot,ev2Dot,n,w,G_all,parameters)

    TT = dot(Td,n)

    return (TT, tau, nTd, ew)

#--------------------------------------------------------------------------#

def UniThrustControl(ep,ev,G_all,parameters):

    u       = cmd_di_3D(ep,ev,parameters);

    g       = G_all[0:3]

    Td      = g - u;

    Tdnorm  = numpy.linalg.norm(Td);

    nTd     = Td/Tdnorm;

    return (u, Td, nTd, Tdnorm)


#--------------------------------------------------------------------------#

# desired acceleration

def UniThrustControlDot(ep,ev,evDot,G_all,parameters):

    g     = G_all[0:3]
    gDot  = G_all[3:6]

    u,Td,nTd,Tdnorm = UniThrustControl(ep,ev,G_all,parameters)

    uDot    = cmd_di_dot_3D(ep,ev,evDot,parameters)


    TdDot   = gDot - uDot;

    nTdDot   = OP(nTd).dot(TdDot)/numpy.linalg.norm(Td);

    TdnormDot  = dot(Td,TdDot)/numpy.linalg.norm(Td);
                  
    return (u,Td,nTd,Tdnorm,uDot,TdDot,nTdDot,TdnormDot)

#--------------------------------------------------------------------------#

def UniThrustControlAngVel(ep,ev,evDot,ev2Dot,n,w,G_all,parameters):


    u,Td,nTd,Tdnorm,uDot,TdDot,nTdDot,TdnormDot = UniThrustControlDot(ep,ev,evDot,G_all,parameters)

    # gradV   = LyapunovGrad(block,ep,ev);
    gradV,Vgrad_grad_ep,Vgrad_grad_ev = LyapunovGrad2_3D(ep,ev,parameters)


    # gains for angular control
    ktt2  = parameters.ktt2
    ktt   = parameters.ktt

    # desired angular velocity
    wd = ktt2/ktt*skew(n).dot(nTd) + skew(nTd).dot(TdDot)/numpy.linalg.norm(Td) - skew(n).dot(gradV)*(numpy.linalg.norm(Td))*1/ktt;

    gradVDot = Vgrad_grad_ep*ev + Vgrad_grad_ev*evDot


    u2Dot    = cmd_di_2dot_3D(ep,ev,evDot,ev2Dot,parameters)

    g2Dot  = G_all[6:9]

    Td2Dot   = g2Dot - u2Dot

    nDot = skew(w).dot(n)

    # wd_dot
    taud = ktt2/ktt*skew(nDot).dot(nTd)

    taud = taud + ktt2/ktt*skew(n).dot(nTdDot)

    taud = taud + skew(nTdDot).dot(TdDot)/Tdnorm
    
    taud = taud + skew(nTd).dot(Td2Dot)/Tdnorm - skew(nTd).dot(TdDot)/(Tdnorm**2)*TdnormDot - skew(nDot).dot(gradV)*(Tdnorm)*1/ktt - skew(n).dot(gradVDot)*(Tdnorm)*1/ktt  - skew(n).dot(gradV)*(TdnormDot)*1/ktt

    kw   = parameters.kw
    kw2  = parameters.kw2

    ew   = wd - w
    tau  = -skew(n).dot(taud) - kw2/kw*skew(n).dot(ew) + ktt/kw*OP(n).dot(nTd) + wd*dot(n,wd);

    return (tau,ew)


#--------------------------------------------------------------------------#
# FOR DOUBLE INTEGRATOR

def cmd_di_3D(ep,ev,parameters):

    u    = numpy.array([0.0,0.0,0.0])
    u[0] = cmd_di(ep[0],ev[0],parameters)
    u[1] = cmd_di(ep[1],ev[1],parameters)
    u[2] = cmd_di(ep[2],ev[2],parameters)
    
    return u
############################################################################

def cmd_di(ep,ev,parameters):
    # command for double integrator

    # gains
    kp = parameters.kp;
    kv = parameters.kv;

    sigma_p  = parameters.sigma_p;
    sigma_v  = parameters.sigma_v;


    h1  = kp*sigma_p*sat(ep/sigma_p);
    h2  = kv*sigma_v*sat(ev/sigma_v);
    f   = fgain(ev/sigma_v);

    u = f*h1 + h2;

    return u

############################################################################

def cmd_di_dot_3D(ep,ev,evD,parameters):

    udot    = numpy.array([0.0,0.0,0.0])
    udot[0] = cmd_di_dot(ep[0],ev[0],evD[0],parameters)
    udot[1] = cmd_di_dot(ep[1],ev[1],evD[1],parameters)
    udot[2] = cmd_di_dot(ep[2],ev[2],evD[2],parameters)

    return udot

############################################################################

def cmd_di_dot(ep,ev,evD,parameters):
    # command for double integrator

    # gains
    kp = parameters.kp
    kv = parameters.kv

    sigma_p  = parameters.sigma_p
    sigma_v  = parameters.sigma_v

    h1  = kp*sigma_p*sat(ep/sigma_p)
    h2  = kv*sigma_v*sat(ev/sigma_v)
    f   = fgain(ev/sigma_v)

    h1Dot = kp*Dsat(ep/sigma_p)*ev
    h2Dot = kv*Dsat(ev/sigma_v)*evD
    fDot  = Dfgain(ev/sigma_v)*evD/sigma_v

    udot  = fDot*h1 + f*h1Dot + h2Dot

    return udot

############################################################################

def cmd_di_2dot_3D(ep,ev,evD,ev2D,parameters):

    u2dot    = numpy.array([0.0,0.0,0.0])
    u2dot[0] = cmd_di_2dot(ep[0],ev[0],evD[0],ev2D[0],parameters)
    u2dot[1] = cmd_di_2dot(ep[1],ev[1],evD[1],ev2D[1],parameters)
    u2dot[2] = cmd_di_2dot(ep[2],ev[2],evD[2],ev2D[2],parameters)

    return u2dot

############################################################################

def cmd_di_2dot(ep,ev,evD,ev2D,parameters):
    # command for double integrator

    # gains
    kp = parameters.kp
    kv = parameters.kv
    
    sigma_p  = parameters.sigma_p
    sigma_v  = parameters.sigma_v

    h1  = kp*sigma_p*sat(ep/sigma_p);
    h2  = kv*sigma_v*sat(ev/sigma_v);
    f   = fgain(ev/sigma_v);

    h1Dot = kp*Dsat(ep/sigma_p)*ev;
    h2Dot = kv*Dsat(ev/sigma_v)*evD;
    fDot  = Dfgain(ev/sigma_v)*evD/sigma_v;

    h12Dot = kp*D2sat(ep/sigma_p)*ev/sigma_p*ev + kp*Dsat(ep/sigma_p)*evD;
    h22Dot = kv*D2sat(ev/sigma_v)*evD/sigma_v*evD + kv*Dsat(ev/sigma_v)*ev2D;
    f2Dot  = D2fgain(ev/sigma_v)*evD/sigma_v*evD/sigma_v + Dfgain(ev/sigma_v)*ev2D/sigma_v;

    u2dot  = f2Dot*h1 + fDot*h1Dot + fDot*h1Dot + f*h12Dot + h22Dot;

    return u2dot

#--------------------------------------------------------------------------#

def LyapunovGrad2_3D(ep,ev,parameters):

    Vgrad         = numpy.array([0.0,0.0,0.0])
    Vgrad_grad_ep = numpy.array([0.0,0.0,0.0])
    Vgrad_grad_ev = numpy.array([0.0,0.0,0.0])

    Vgrad[0],Vgrad_grad_ep[0],Vgrad_grad_ev[0] = LyapunovGrad2(ep[0],ev[0],parameters)
    Vgrad[1],Vgrad_grad_ep[1],Vgrad_grad_ev[1] = LyapunovGrad2(ep[1],ev[1],parameters)
    Vgrad[2],Vgrad_grad_ep[2],Vgrad_grad_ev[2] = LyapunovGrad2(ep[2],ev[2],parameters)

    return (Vgrad,Vgrad_grad_ep,Vgrad_grad_ev)

############################################################################

def LyapunovGrad2(ep,ev,parameters):

    # Vgrad         = dV/d(ev)
    # Vgrad_grad_ep = d/d(ep) [dV/d(ev)]
    # Vgrad_grad_ev = d/d(ev) [dV/d(ev)]

    # gains
    kp = parameters.kp
    kv = parameters.kv
    
    sigma_p  = parameters.sigma_p
    sigma_v  = parameters.sigma_v

    beta   = 1.0/(2.0*kp)
    h1     = kp*sigma_p*sat(ep/sigma_p)
    h1D    = kp*Dsat(ep/sigma_p)
    h2     = kv*sigma_v*sat(ev/sigma_v)
    h2D    = kv*Dsat(ev/sigma_v)
    h22D   = kv*D2sat(ev/sigma_v)/sigma_v
    f      = fgain(ev/sigma_v)
    fD     = Dfgain(ev/sigma_v)/sigma_v

    Vgrad = beta*h1*h2D + ev/f + beta/f*((kv**2)*ev - h2*h2D)

    Vgrad_grad_ep = beta*h1D*h2D

    Vgrad_grad_ev = beta*h1*h22D + 1.0/f - ev/(f**2)*fD - beta/(f**2)*fD*((kv**2)*ev - h2*h2D) + beta/f*(kv**2 - h2D*h2D - h2*h22D)

    return (Vgrad,Vgrad_grad_ep,Vgrad_grad_ev)

#--------------------------------------------------------------------------#




def sat(x):

    return x/numpy.sqrt(1 + x**2.0)


def Dsat(x):

    return 1.0/numpy.power(1 + x**2.0,1.5)


def D2sat(x):

    return -3.0*x/numpy.power(1.0+x**2.0, 2.5)


def sat_Int(x):
    # primitive of saturation function

    return numpy.sqrt(1.0 + x**2.0) - 1.0;



def fgain(x):

    return 1.0/numpy.sqrt(1.0 + x**2.0)


def Dfgain(x):

    return -x/numpy.power(1.0 + x**2.0, 1.5);


def D2fgain(x):

    return (-1.0 + 2.0*x**2.0)/numpy.power(1.0 + x**2.0, 2.5);


def fgain_int(x):

    # integral of x/fgain(x) from 0 to in

    return 1.0/3.0*(-1.0 + numpy.power(1.0 + x**2.0, 1.5));

def fgain_int2(x):

    # integral of sat(x)*Dsat(x)/fgain(x) from 0 to x

    return 1.0 - 1.0/sqrt(1.0 + x**2.0);


#--------------------------------------------------------------------------#
# ParametersSystem
class ParametersSys(object):

    # transported mass (kg)
    M  = 0.01
        
    # acceleration due to gravity (m/s^2)
    g  = 9.81
        

    # mass of vehicles (kg)
    m = 1.5
        
    # cable lengths (m)
    L = 5.0

    # controller gains
    ktt  = 4.0
    ktt2 = 6.0

    kw   = 10.0
    kw2  = 14.0
        
    kv   = 2.0
    kp   = 1.0/numpy.sqrt(2)*2.0

    sigma_p = 0.5
    sigma_v = 0.5


    # The class "constructor" - It's actually an initializer
    # def __init__(self):
    #   self.M = 1.1


    def make_par():
        parameters = ParametersSys()
        return parameters

