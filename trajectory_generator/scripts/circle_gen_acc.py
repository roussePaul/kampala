#!/usr/bin/env python
import rospy
import sys
import ast
import math
from mocap.msg import QuadPositionDerived
from controller.msg import Permission
from straight_line_class import StraightLineGen
from trajectory_generato import TrajectoryGenerator

#This script generates the points, velocities and accelerations to be used as a reference for the 
#controller to to get the quad to move in a circle.
#Given a midpoint, radius and speed a circle is generated such that the quad moves at a constant speed.
#Constraints on maximum velocity and acceleration are used.
#Everyhthing is calculated in a coordinatesystem that is rotated by an angle of theta about the z-axis of the SML-frame. The method transform_coordinates transforms a vector given in the rotated frame into the corresponding vector in the SML-frame. 

class CircleGen:
  
  def __init__(self):
    self.done = False
    self.midpoint = rospy.get_param("trajectory_generator/midpoint",[0.0,0.0,0.6])
    if type(self.midpoint) is str:
      self.midpoint = ast.literal_eval(self.midpoint)
    self.radius = rospy.get_param("trajectory_generator/radius",0.8)
    self.velo = rospy.get_param("trajectory_generator/velo",0.4)
    self.theta = rospy.get_param("trajectory_generator/theta",0)
  def get_tilted_circle(self):
    tilted_midpoint = self.inverse_transform(self.midpoint,self.theta) 
    a_max = 0.6**2.0/0.8
    #v_max = (self.radius*a_max)**(0.5)
    #if self.velo >= v_max:
     # self.velo = 0.9*v_max
    start_point = self.go_to_start(tilted_midpoint)
    pub = rospy.Publisher('trajectory_gen/target',QuadPositionDerived, queue_size=10)
    sec_pub = rospy.Publisher('trajectory_gen/done', Permission, queue_size=10)
    rospy.init_node('TG',anonymous=True)
    rospy.sleep(4.)
    r = 10.0
    rate = rospy.Rate(r)
    t_f = self.velo/math.sqrt(a_max**2.0 - self.velo**4.0/self.radius**2.0)
    time = self.accelerate(pub, a_max, r, tilted_midpoint)/2.0 - 1/r
    period = 2*math.pi*self.radius/self.velo
    points_in_period = int(period * r)
    counter = 0
    while not rospy.is_shutdown() and not self.done:
      out_pos = self.transform_coordinates(self.get_outpos(self.radius, self.velo, tilted_midpoint, time),self.theta)
      out_velo = self.transform_coordinates(self.get_outvelo(self.radius, self.velo, time),self.theta)
      out_acc = self.transform_coordinates(self.get_outacc(self.radius, self.velo, time),self.theta)
      out_msg = QuadPositionDerived()
      out_msg.x = out_pos[0]
      out_msg.y = out_pos[1]
      out_msg.z = out_pos[2]
      out_msg.yaw = 0
      out_msg.x_vel = out_velo[0]  
      out_msg.y_vel = out_velo[1]
      out_msg.z_vel = out_velo[2]
      out_msg.yaw_vel = 0
      out_msg.x_acc = out_acc[0]
      out_msg.y_acc = out_acc[1]
      out_msg.z_acc = out_acc[2]
      out_msg.yaw_acc = 0
      pub.publish(out_msg)
      sec_pub.publish(Permission(False))
      time += 1.0/r
      counter += 1
      rate.sleep()
      if time >= 3*period - t_f/2:
        end_pos = self.decallerate(pub,a_max,self.radius, self.velo, tilted_midpoint, time)
        rospy.set_param("trajectory_generator/start_point",end_pos)
        rospy.set_param("trajectory_generator/end_point",[0.0,0.0,0.6])
        rospy.sleep(2.)
        sl_gen = StraightLineGen() 
        rospy.sleep(4.)
        sl_gen.generate()
        rospy.sleep(4.)
        #self.turn(pub, [0.0,0.0,0.6],0)
        self.done = True
    sec_pub.publish(Permission(True))
    
    
    


  def get_outpos(self, R,v,r_m,t):
    outpos = [0.0,0.0,0.0]
    outpos[0] = r_m[0] + R*math.cos(v*t/R)
    outpos[1] = r_m[1] + R*math.sin(v*t/R)
    outpos[2] = r_m[2]
    return outpos

  def get_outvelo(self,R,v,t):
    outvelo = [0.0,0.0,0.0]
    outvelo[0] = -v*math.sin(v*t/R)
    outvelo[1] = v*math.cos(v*t/R)
    outvelo[2] = 0
    return outvelo

  def get_outacc(self,R,v,t):
    outacc = [0.0,0.0,0.0]
    outacc[0] = -v**2.0/R*math.cos(v*t/R)
    outacc[1] = -v**2.0/R*math.sin(v*t/R)
    outacc[2] = 0
    return outacc

  def transform_coordinates(self,coordinates,theta):             #transforms from tilted frame to SML-frame
    x_temp = coordinates[0] * math.cos(theta) + coordinates[2] * math.sin(theta)
    z_temp = coordinates[2] * math.cos(theta) - coordinates[0] * math.sin(theta)
    return [x_temp,coordinates[1],z_temp]  

  def inverse_transform(self,coordinates,theta):  #transforms from SML-frame to tilted frame
    x_temp = coordinates[0] * math.cos(theta) - coordinates[2] * math.sin(theta)
    z_temp = coordinates[0] * math.sin(theta) + coordinates[2] * math.cos(theta)
    return [x_temp, coordinates[1], z_temp]

  def get_yaw(self,v,t,R):
    yaw = math.pi/2 + v*t/R
    is_positive = yaw > 0 
    if is_positive:
      while yaw > math.pi:
        yaw -= 2*math.pi
    else:
      while yaw < -(math.pi):
        yaw += 2*math.pi
    return 0    

  def go_to_start(self,midpoint):
    target_point = [0.0,0.0,0.0]
    target_point[0] = midpoint[0] + self.radius
    target_point[1] = midpoint[1]
    target_point[2] = midpoint[2]
    outpos = self.transform_coordinates(target_point,self.theta)
    rospy.set_param("trajectory_generator/start_point",[0.0,0.0,0.2])
    rospy.set_param("trajectory_generator/end_point",outpos)
    sl_gen = StraightLineGen()
    sl_gen.generate()
    return outpos
    

  def turn(self,pub, pos ,yaw):
    out_msg = QuadPositionDerived()
    out_msg.x = pos[0]
    out_msg.y = pos[1]
    out_msg.z = pos[2]
    out_msg.yaw = yaw
    out_msg.x_vel = 0  
    out_msg.y_vel = 0
    out_msg.z_vel = 0
    out_msg.yaw_vel = 0
    out_msg.x_acc = 0
    out_msg.y_acc = 0
    out_msg.z_acc = 0
    out_msg.yaw_acc = 0
    pub.publish(out_msg)
    rospy.sleep(1)
    
  def accelerate(self, pub, a_max, r, tilted_midpoint):
    t_f = self.velo/math.sqrt(a_max**2.0 - self.velo**4.0/self.radius**2.0)
    rate = rospy.Rate(r)
    time = 0.0  
    done = False
    while not rospy.is_shutdown() and not done: 
      outpos = self.transform_coordinates(self.get_outpos_accphase(self.radius, self.velo, tilted_midpoint, time, t_f),self.theta)
      outvelo = self.transform_coordinates(self.get_outvelo_accphase(self.radius, self.velo, time, t_f),self.theta) 
      outacc = self.transform_coordinates(self.get_outacc_accphase(self.radius, self.velo, time, t_f),self.theta)
      out_msg = QuadPositionDerived()
      out_msg.x = outpos[0]
      out_msg.y = outpos[1]
      out_msg.z = outpos[2]
      out_msg.yaw = 0
      out_msg.x_vel = outvelo[0]  
      out_msg.y_vel = outvelo[1]
      out_msg.z_vel = outvelo[2]
      out_msg.yaw_vel = 0
      out_msg.x_acc = outacc[0]
      out_msg.y_acc = outacc[1]
      out_msg.z_acc = outacc[2]
      out_msg.yaw_acc = 0
      pub.publish(out_msg)
      rate.sleep()
      time += 1/r
      if time > t_f:
        done  = True
    return time

  def get_outpos_accphase(self, R,v,r_m,t, t_f):
    outpos = [0.0,0.0,0.0]
    outpos[0] = r_m[0] + R*math.cos(v*t**2.0/(2.0*R*t_f))
    outpos[1] = r_m[1] + R*math.sin(v*t**2.0/(2.0*R*t_f))
    outpos[2] = r_m[2]
    return outpos

  def get_outvelo_accphase(self,R,v,t, t_f):
    outvelo = [0.0,0.0,0.0]
    outvelo[0] = -v*t/t_f*math.sin(v*t**2.0/(2.0*R*t_f))
    outvelo[1] = v*t/t_f*math.cos(v*t**2.0/(2.0*R*t_f))
    outvelo[2] = 0
    return outvelo

  def get_outacc_accphase(self,R,v,t,t_f):
    outacc = [0.0,0.0,0.0]
    outacc[0] = -v/t_f*math.sin(v*t**2.0/(2.0*R*t_f)) - v**2.0 * t**2.0/(R*t_f**2.0) * math.cos(v*t**2.0/(2.0*R*t_f))
    outacc[1] = v/t_f*math.cos(v*t**2.0/(2.0*R*t_f)) - v**2.0 * t**2.0/(R*t_f**2.0) * math.sin(v*t**2.0/(2.0*R*t_f))
    outacc[2] = 0
    return outacc
   
   
  def decallerate(self,pub,a_max,R, v,tilted_midpoint,time):  
    t_f = self.velo/math.sqrt(a_max**2.0 - self.velo**4.0/self.radius**2.0)
    w_0 = v/R
    theta_0 = v/R*time
    r = 10.0
    rate = rospy.Rate(r)
    time = 1/r
    outpos = [0.0,0.0,0.0]
    while time <= t_f:
      outpos = self.transform_coordinates(self.get_outpos_deaccphase(self.radius, self.velo, tilted_midpoint, time, t_f, theta_0,w_0),self.theta)
      outvelo = self.transform_coordinates(self.get_outvelo_deaccphase(self.radius, self.velo, time, t_f,theta_0, w_0),self.theta) 
      outacc = self.transform_coordinates(self.get_outacc_deaccphase(self.radius, self.velo, time, t_f,theta_0, w_0),self.theta)
      out_msg = QuadPositionDerived()
      out_msg.x = outpos[0]
      out_msg.y = outpos[1]
      out_msg.z = outpos[2]
      out_msg.yaw = 0
      out_msg.x_vel = outvelo[0]  
      out_msg.y_vel = outvelo[1]
      out_msg.z_vel = outvelo[2]
      out_msg.yaw_vel = 0
      out_msg.x_acc = outacc[0]
      out_msg.y_acc = outacc[1]
      out_msg.z_acc = outacc[2]
      out_msg.yaw_acc = 0
      pub.publish(out_msg)
      rate.sleep()
      time += 1/r
    return outpos
 
    
  def get_outpos_deaccphase(self, R,v,r_m,t, t_f,theta_0,w_0):
    outpos = [0.0,0.0,0.0]
    outpos[0] = r_m[0] + R*math.cos(theta_0+w_0*t-v*t**2.0/(2.0*R*t_f))
    outpos[1] = r_m[1] + R*math.sin(theta_0+w_0*t-v*t**2.0/(2.0*R*t_f))
    outpos[2] = r_m[2]
    return outpos

  def get_outvelo_deaccphase(self,R,v,t, t_f,theta_0, w_0):
    outvelo = [0.0,0.0,0.0]
    outvelo[0] = (R*w_0-v*t/t_f)*math.sin(theta_0+w_0*t-v*t**2.0/(2.0*R*t_f))
    outvelo[1] = -(R*w_0-v*t/t_f)*math.cos(theta_0+w_0*t-v*t**2.0/(2.0*R*t_f))
    outvelo[2] = 0
    return outvelo

  def get_outacc_deaccphase(self,R,v,t,t_f,theta_0, w_0):
    outacc = [0.0,0.0,0.0]
    outacc[0] = -v/t_f*math.sin(theta_0+w_0*t-v*t**2.0/(2.0*R*t_f)) - R * (w_0-v*t/(R*t_f))**2.0 * math.cos(theta_0+w_0*t-v*t**2.0/(2.0*R*t_f))
    outacc[1] = v/t_f*math.cos(theta_0+w_0*t-v*t**2.0/(2.0*R*t_f)) - R * (w_0-v*t/(R*t_f))**2.0 * math.sin(theta_0+w_0*t-v*t**2.0/(2.0*R*t_f))
    outacc[2] = 0
    return outacc

if __name__ == '__main__':
  rospy.sleep(5.)
  try:
    circle_generator = CircleGen()
    circle_generator.get_tilted_circle()  
  except rospy.ROSInterruptException:
    pass
