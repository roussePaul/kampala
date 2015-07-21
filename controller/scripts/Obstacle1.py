#!/usr/bin/env python

import rospy
import math
import numpy as np
from straight_line_class import StraightLineGen
from bend_class import CircleGen
from numpy import linalg as LA

def Inside_sphere(x,y,z,xc,yc,zc,r):
    return ((x-xc)**2.0 + (y-yc)**2.0 + (z-zc)**2.0)<=r**2.0

def get_data():

    #mocap_get_data = rospy.ServiceProxy('mocap_get_data',MocapGetData)
    #data = mocap_get_data(1)
    #return data
    return [-1.5,1.4,1.3]

def Calculate_params(start,end,r,n):

    
    data = get_data()
    xc = data[0]
    yc = data[1]
    zc = data[2]
    
    xa = start[0]
    ya = start[1]
    za = start[2]
    
    deltax = (end[0] - xa)/n
    deltay = (end[1] - ya)/n 
    deltaz = (end[2] - za)/n

    enterj=n+1
    exitj=n+1

    for j in range(0,n+1):
    
        xtemp = xa + j*deltax
        ytemp = ya + j*deltay
        ztemp = za + j*deltaz 
        
        if Inside_sphere(xtemp,ytemp,ztemp,xc,yc,zc,r):
            enterj = j
            break
    
    for j in range(enterj,n+1):
        
        xtemp = xa + j*deltax
        ytemp = ya + j*deltay
        ztemp = za + j*deltaz
   
        if not Inside_sphere(xtemp,ytemp,ztemp,xc,yc,zc,r):
            exitj = j
            break

    
    xenter = xa + deltax*enterj
    yenter = ya + deltay*enterj
    zenter = za + deltaz*enterj

    xexit = xa + deltax*exitj
    yexit = ya + deltay*exitj
    zexit = za + deltaz*exitj

    enterpoint = [xenter,yenter,zenter]
    exitpoint = [xexit,yexit,zexit]

    r1 = [xenter-xc,yenter-yc,zenter-zc]
    r2 = [xexit-xc,yexit-yc,zexit-zc]
    print np.dot(r1,r2)/(LA.norm(r1)*LA.norm(r2))
    
    cospsi = np.dot(r1,r2)/(LA.norm(r1)*LA.norm(r2))
    if cospsi > 1:
        psi = 0
    elif cospsi < -1:
        psi = math.pi
    else:
        psi = math.acos(cospsi)

    return [psi,enterpoint,exitpoint,r1,r2]
    
    

if __name__ == '__main__':

    rospy.set_param("trajectory_generator/start_point",[0.0,0.0,0.02
])
    rospy.set_param("trajectory_generator/end_point" , [-3.0,3.0,3.0])
    rospy.set_param('obstacle_radius',0.5)
    rospy.set_param('n',1000)

    centerpoint = get_data()
    start = rospy.get_param("trajectory_generator/start_point",[0.0,0.0,0.01])
    end = rospy.get_param("trajectory_generator/end_point" , [-1.0,-1.2,1.05])
    r = rospy.get_param('obstacle_radius',math.sqrt(2)*0.25)
    n = rospy.get_param('n',1000)
    
    if not Inside_sphere(start[0],start[1],start[2],centerpoint[0],centerpoint[1],centerpoint[2],r):
	params = Calculate_params(start,end,r,n)
	psi = params[0]
	enterpoint = params[1]
	exitpoint = params[2]
	r1 = params[3]
	r2 = params[4]
	initialvelo = np.cross(np.cross(r1,r2),r1)
	norm = LA.norm(initialvelo)
	if norm!=0:
	    initialvelo = (initialvelo*0.283/LA.norm(initialvelo)).tolist()
	else:
	    initialvelo = [0.0,0.0,0.283]
	print initialvelo


	rospy.set_param("trajectory_generator/end_point",enterpoint)
	linegen = StraightLineGen()
	linegen.generate()

        if not Inside_sphere(end[0],end[1],end[2],centerpoint[0],centerpoint[1],centerpoint[2],r):

	    if enterpoint != exitpoint:
	        rospy.set_param("trajectory_generator/midpoint",centerpoint)
	        rospy.set_param("trajectory_generator/start",enterpoint)
	        rospy.set_param("trajectory_generator/psi",psi)
	        rospy.set_param("trajectory_generator/initial_velo",initialvelo)
	        circlegen = CircleGen()
	        circlegen.get_tilted_circle()

	    rospy.set_param("trajectory_generator/start_point",exitpoint) 
	    rospy.set_param("trajectory_generator/end_point",end)
	    linegen = StraightLineGen() 
	    linegen.generate()
    
