#!/usr/bin/env python
import rospy
import sys
import ast
import math
from mocap.msg import QuadPositionDerived
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, PoseStamped, Quaternion

#No respect is taken to yaw yet
#Given points A and B and the origin O a straight line is generated between A and B of the form
# r(t)=OA+s(t)*e_t, where e_t = (OB - OA)/||OB - OA||, s(0) = 0 , s(t_f) = ||OB - OA||, ds/dt(0) = 0, ds/dt(t_f) = 0, t_f = time to move from A to B, s(t) = t^2*(t_f/2-t/3)
#To satisfy the constraints on s and ds/dt it is needed that t_f = (6||OB - OA||)^(1/3)
#Constraints on acceleration and velocity lead to constraints on t_f and hence ||OB - OA||
#To be able to go between any two points an intermediate point is generated whenever the constraint on ||OB - OA|| is not met
#In the terminal two lists of the form [x,y,z] representing OA and OB must be given


def starter():
  start_point = rospy.get_param("trajectory_generator/start_point",[0.0,0.0,0.6])
  end_point = rospy.get_param("trajectory_generator/end_point",[0.0,0.5,0.6])
  if type(start_point is str):
    start_point = ast.literal_eval(start_point)
  if type(end_point is str):
    end_point = ast.literal_eval(end_point)
  a_max = 10.0
  v_max = 10.0
  i = 0
  inter_points = []
  norm = ((end_point[0] - start_point[0])**2 + (end_point[1] - start_point[1])**2 + (end_point[2] - start_point[2])**2)**(1.0/2.0)
  pub = rospy.Publisher('trajectory_gen/target',QuadPositionDerived, queue_size=10)
  pubviz = rospy.Publisher('trajectory_gen/visualization',Path, queue_size=10)
  viz_list = Path()
  viz_list.header.frame_id = "/map"
  rospy.init_node('ptp_generator',anonymous=True)
  r = 25
  rate = rospy.Rate(r)
  for b in range(0,15*r):
    pub.publish(QuadPositionDerived(0,0,0.6,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0))
    rate.sleep()
  if norm == 0:
    while not rospy.is_shutdown():
      rate = rospy.Rate(25)
      pub.publish(QuadPositionDerived(end_point[0],end_point[1],end_point[2],0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0))
      pose = Pose(Point(end_point[0], end_point[1], end_point[2]), Quaternion())
      viz_list.poses.append(PoseStamped(Header(0, rospy.Time.now(), "/map"), pose))
      pubviz.publish(viz_list)
      rate.sleep()
  e_t = [(end_point[0] - start_point[0])/norm,(end_point[1] - start_point[1])/norm,(end_point[2] - start_point[2])/norm]
  yaw = math.acos(e_t[0])
  num_of_points = num_of_new_points(norm,a_max,v_max)
  temp = False
  if num_of_points != 0:         
    inter_points = [0]*num_of_points
    temp = True
  while  i < num_of_points and temp:
    if i == 0:
      inter_points[i] = find_new_point(start_point,a_max,v_max,e_t)
      i += 1
    else:
      inter_points[i] = find_new_point(inter_points[i-1],a_max,v_max,e_t)
      i += 1
  for j in range(0,num_of_points-1):
    ptp_generator(inter_points[j],inter_points[j+1],e_t,pub,pubviz,viz_list,False,yaw)
    
  if temp:
    ptp_generator(inter_points[num_of_points-1],end_point,e_t,pub,pubviz,viz_list,True,yaw)
  else:
    ptp_generator(start_point, end_point,e_t,pub,pubviz,viz_list,True,yaw)
  

def ptp_generator(start_point,end_point,e_t,pub,pubviz,viz_list,is_end_point,yaw):
  a_max = 10.0
  v_max = 10.0
  n = 35
  k = 0
  norm = ((end_point[0] - start_point[0])**2 + (end_point[1] - start_point[1])**2 + (end_point[2] - start_point[2])**2)**(1.0/2.0)
  t_f = (6.0*norm)**(1.0/3.0)
  r = n/t_f + 1/(10*t_f) #publishing frequency
  rate = rospy.Rate(r) 
  s = [0.0] * (n+1)
  s_d = [0.0] * (n+1)
  s_dd = [0.0] * (n+1)
  for j in range(1,n+1):
    s[j] =  (j/r+1/(10*r))**2.0 * (t_f/2.0 - (j/r+1/(10.0*r))/3.0)
    s_d[j] =  (j/r+1/(10.0*r)) * t_f -  (j/r+1/(10.0*r))**2.0 
    s_dd[j] = ( t_f - 2 * (j/r+1/(10.0*r))) 
  outpos = [0.0]*3
  outvelo = [0.0]*3
  outacc = [0.0]*3
  while not rospy.is_shutdown():
    if k <= n:
      outpos = get_outpos(start_point,s[k], e_t)
      outvelo = get_outvelo(s_d[k], e_t)
      outacc = get_outacc(s_dd[k], e_t)
      out_msg = QuadPositionDerived(outpos[0],outpos[1],outpos[2],yaw,outvelo[0],outvelo[1],outvelo[2],0.0,outacc[0],outacc[1],outacc[2],0.0,0.0)
      pub.publish(out_msg)
      pose = Pose(Point(outpos[0], outpos[1], outpos[2]), Quaternion())
      viz_list.poses.append(PoseStamped(Header(0, rospy.Time.now(), "/map"), pose))
      pubviz.publish(viz_list)
      k += 1
      rate.sleep()
    elif is_end_point:
      pub.publish(QuadPositionDerived(end_point[0],end_point[1],end_point[2],yaw,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0))
      pose = Pose(Point(end_point[0], end_point[1], end_point[2]), Quaternion())
      viz_list.poses.append(PoseStamped(Header(0, rospy.Time.now(), "/map"), pose))
      pubviz.publish(viz_list)
      rate.sleep()  
    else: 
      return

def get_outpos(start_point, s, e_t):
  outpos = [0.0,0.0,0.0]
  for i in range(0,3):
    outpos[i] = start_point[i] + s * e_t[i]             
  return outpos

def get_outvelo(s_d, e_t):
  outvelo = [0.0,0.0,0.0]
  for i in range(0,3):
    outvelo[i] = s_d*e_t[i]
  return outvelo

def get_outacc(s_dd, e_t):
  outacc = [0.0,0.0,0.0]
  for i in range(0,3):
    outacc[i] = s_dd*e_t[i]
  return outacc

def find_new_point(start,a,v,e_t):
  dist = min(a**3.0/6,v**(1.5)*4.0/3.0)*0.9
  new_point = [0,0,0]
  for i in range (0,3):
    new_point[i] = start[i] + dist * e_t[i]
  return new_point
  
def num_of_new_points(norm,a,v):
  dist = min(a**3.0/6,v**(1.5)*4.0/3.0)*0.9
  n = 0
  length = norm
  while length > dist:
    length -= dist
    n += 1
  return n
  

if __name__ == '__main__':
  #args = rospy.myargv(argv=sys.argv)
  #start_point = ast.literal_eval(args[1])
  #end_point = ast.literal_eval(args[2])
  try:
    starter()
  except rospy.ROSInterruptException:
    pass
