#!/usr/bin/env python

import control
import copy
import math
import numpy as np
class System:
    def __init__(self,sys,X_0=[]):
        s = control.tf([1.0,0.0],[1.0])

        self.sys = control.tf2ss(s/s* copy.deepcopy(sys) )
  
        self.issys = True
        w = control.pzmap.pzmap(self.sys)
        if w[0].size==0 and w[0].size==0:
            self.sys = sys.num[0][0][0]/sys.den[0][0][0]
            self.issys = False
        else:
            if X_0==[]:
                T = [0,0.001]
                yout,T,X = control.matlab.lsim(self.sys,T=T)
                X_0 = X[0]

        self.X = np.transpose([X_0])
        self.T = -1.0
        self.I = 0

        self.nextX = self.X
        self.nextT = self.T
        self.nextI = self.I

    def output2(self, U, time):
        U = copy.deepcopy(U)
        time = copy.deepcopy(time)
        if self.issys==False:
            self.T = time
            return self.sys*U

        if self.T==-1:
            self.T = time
            self.nextT = time
            return 0.0

        T = [self.T,time]
        I = [self.I,U]

        # attr = vars(self)
        # print "---------------------------------------------"
        # for (k,v) in attr.iteritems():
        #     print k
        #     print v
        #     print type(v)
        #     print
        # print "---------------------------------------------"
        # print T
        # print type(T)
        # print I
        # print type(I)
        # print U
        # print type(U)
        # print self.X
        # print type(self.X)
        yout,T,X = control.matlab.lsim(self.sys,T=T,U=I,X0=self.X)

        self.nextT = time
        self.nextX = X[1]
        self.nextI = U

        return yout[1]

    def output(self,U,time):

        if self.T==-1:
            self.T = time
            self.nextT = time
            return 0.0

        A = self.sys.A
        B = self.sys.B
        C = self.sys.C
        D = self.sys.D

        M = A.shape[0]

        # Get time difference
        dt = time-self.T
        self.nextT = time

        I = np.identity(M)
        Ai = np.linalg.inv(I-dt/2.0*A)
        phi = Ai*(I+dt/2.0*A)
        gamma = Ai*dt*B

        # Apply the filter

        self.nextX = phi*self.X + gamma*1/2.0*(U+self.I)
        self.nextI = U

        out = C*self.nextX + D*U

        return out.item((0,0))

    def next_state(self):
        self.X = self.nextX
        self.T = self.nextT
        self.I = self.nextI

    def freeze_state(self):
        self.T = self.nextT
        self.I = self.nextI



if __name__=="__main__":
    import random
    s = control.tf([1.0,0.0],[1.0])
    wc = 1.0*2.0*math.pi
    d = s/(1+s/wc)
    acc = System( d**2 )
    vel = System( d )
    t=0.0
    while t<1.0:
        t += 1.0/30.0
        i = t**2 + random.normalvariate(0,0.01)
        print acc.output(i,t)

        #print ', '.join("%s: %s" % item for item in attrs.items())
        vel.next_state()
        acc.next_state()
