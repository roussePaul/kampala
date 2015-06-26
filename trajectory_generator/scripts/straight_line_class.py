#!/usr/bin/env python
import rospy
import sys
import ast
import math
from controller.msg import Permission
from mocap.msg import QuadPositionDerived

#generates a straight line between startpoint and endpoint with velocity zero at the start and endpoint
#respects constraints on acceleration

class StraightLineGen:
 
  def __init__(self,start=[0.0,0.0,0.0],end=[0.0,0.0,0.6]):
    print("uber")
    self.done = False
    self.killed = False
    self.start_point = rospy.get_param("trajectory_generator/start_point",start)
    self.end_point =  rospy.get_param("trajectory_generator/end_point",end)
    if type(self.start_point) is str:
      self.start_point = ast.literal_eval(self.start_point)
    if type(self.end_point) is str:
      self.end_point = ast.literal_eval(self.end_point)
    self.sec_pub = rospy.Publisher('trajectory_gen/done',Permission,queue_size=10)

  def generate(self):
    a_max = 9.81/3.0
    dist = self.get_distance(self.start_point, self.end_point)
    e_t = self.get_direction(self.start_point, self.end_point)
    t_f = math.sqrt(6*dist/0.9*a_max)
    constant = -2.0/t_f**3.0 * dist
    rospy.init_node('TG')
    pub = rospy.Publisher('trajectory_gen/target',QuadPositionDerived, queue_size=10)
    r = 10
    rate = rospy.Rate(r)
    time = 0.0
    while not rospy.is_shutdown() and not self.done:      #OBS: exits when the final position is reached
      if time < t_f:
        outpos = self.get_position(self.start_point, dist, e_t, time, t_f, constant)
        outvelo = self.get_velocity(dist, e_t, time, t_f, constant)
        outacc = self.get_acceleration(dist, e_t, time, t_f, constant)
        yaw = 0
        out_msg = self.get_message(outpos, 0, outvelo, outacc)
        pub.publish(out_msg)
        self.sec_pub.publish(Permission(False))
        time += 1.0/r
        rate.sleep()
      else:
        out_msg = self.get_message(self.end_point, 0, [0.0,0.0,0.0], [0.0,0.0,0.0])
        self.sec_pub.publish(Permission(False))
        pub.publish(out_msg)
        self.done = True
        rate.sleep()

  def get_distance(self,a,b):
    dist_squared = 0
    for i in range(0,3):
      dist_squared += (a[i] - b[i])**2.0
    dist = math.sqrt(dist_squared)
    return dist

  def get_message(self,outpos, yaw, outvelo, outacc):
    msg = QuadPositionDerived()
    msg.x = outpos[0]
    msg.y = outpos[1]
    msg.z = outpos[2]
    msg.yaw = 0
    msg.x_vel = outvelo[0]
    msg.y_vel = outvelo[1]
    msg.z_vel = outvelo[2]
    msg.yaw_vel = 0
    msg.x_acc = outacc[0]
    msg.y_acc = outacc[1]  
    msg.z_acc = outacc[2]
    msg.yaw_acc = 0
    return msg  
  

  def get_direction(self,a, b):
    norm = self.get_distance(a,b)
    e_t = [0.0,0.0,0.0]
    for j in range(0,3):
      e_t[j] = (b[j] - a[j])/norm
    return e_t
  
  def get_position(self,start, dist, e_t, t, t_f, c):
    outpos = [0.0,0.0,0.0]
    s = self.get_s(t, t_f, c)
    for i in range(0,3):
      outpos[i] = start[i] + s * e_t[i]
    return outpos

  def get_s(self,t, t_f, c):
    return t**2.0 * c * (t-1.5*t_f)

  def get_yaw(self,e_t):
    yaw = math.acos(e_t[0])
    #return math.degrees(yaw)
    return 0

  def get_velocity(self,dist, e_t, t, t_f, c):
    outvelo = [0.0,0.0,0.0]
    s_d = self.get_s_d(t, t_f, c)
    for i in range(0,3):
      outvelo[i] = s_d * e_t[i]
    return outvelo

  def get_s_d (self,t, t_f, c):
    return c * (3 * t**2.0 - 3 * t_f * t)

  def get_acceleration(self,dist, e_t, t, t_f, c):
    outacc = [0.0,0.0,0.0]
    s_dd = self.get_s_dd(t, t_f, c)
    for i in range(0,3):
      outacc[i] = s_dd * e_t[i]
    return outacc

  def get_s_dd (self,t, t_f, c):
    return c * (6 * t - 3 * t_f )
  
  def set_done(self, boolean):
    self.done = boolean  

  def is_done(self):
    return self.done
  
  def set_killed(self, boolean):
    self.killed = boolean

  def is_killed(self):
    return self.killed

  def adjust_yaw(self,pub, e_t):
    out = self.get_message(self.start_point, self.get_yaw(e_t), [0,0,0], [0,0,0])
    pub.publish(out)

  def land(self):
    self.sec_pub.publish(True)



if __name__ == '__main__':
  rospy.sleep(5)
  try:
    StraightLineGen().generate()
  except rospy.ROSInterruptException:
    pass







  
