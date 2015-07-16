#!/usr/bin/env python

import control

class System:
    def __init__(self,sys,X_0=[]):
        self.sys = sys

        self.issys = True
        w = control.pzmap.pzmap(self.sys)
        if w[0].size==0 and w[0].size==0:
            self.sys = sys.num[0][0][0]/sys.den[0][0][0]
            self.issys = False
        else:
            print sys
            if X_0==[]:
                T = [0,0.001]
                yout,T,X = control.matlab.lsim(self.sys,T=T)
                X_0 = X[0]

        self.X = X_0
        self.T = -1.0
        self.I = 0

        self.nextX = self.X
        self.nextT = self.T
        self.nextI = self.I

    def output(self, U, time):
        if self.issys==False:
            self.T = time
            return self.sys*U

        if self.T==-1:
            self.T = time
            self.nextT = time
            return 0.0

        T = [self.T,time]
        I = [self.I,U]

        yout,T,X = control.matlab.lsim(self.sys,T=T,U=I,X0=self.X)

        self.nextT = time
        self.nextX = X[1]
        self.nextI = U

        return yout[1]

    def next_state(self):
        self.X = self.nextX
        self.T = self.nextT
        self.I = self.nextI

    def freeze_state(self):
        self.T = self.nextT
        self.I = self.nextI



if __name__=="__main__":
    s = System( control.tf([1.0],[1.0,0.0,0.0]) )

    t=0.0
    while t<1.0:
        t += 0.03
        print s.output(1.0,t)

        attrs = vars(s)
        #print ', '.join("%s: %s" % item for item in attrs.items())
        s.next_state()


    while t<2.0:
        t += 0.03
        print s.output(1.0,t)

        attrs = vars(s)
        #print ', '.join("%s: %s" % item for item in attrs.items())
        s.freeze_state()


    s =  control.tf([1.0,0.0],[1.0])
    print s
    T = [0,0.001]
    test = System(1+s/(1+s))
