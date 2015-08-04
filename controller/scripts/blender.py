#!/usr/bin/env python

# Blender script for the IRIS+ in the SML Lab 
# Gets inputs from the Security Guard and the Trajectory Generator
# Publishes commands via Mavros' rc/override topic
 

import rospy
import sml_setup
import sys
import math
from copy import deepcopy
import numpy
from controller_base import Controller
from PID_controller import PID
from load_transport_controller import LoadTransportController
from point import *
from controller.msg import PlotData
from controller.srv import SetChannel6
from mavros.msg import OverrideRCIn
from mocap.msg import QuadPositionDerived, QuadPositionDerivedExt
from controller.msg import Permission
from obstacle_avoidance import AvoidanceController
from std_srvs.srv import Empty
import utils
from std_msgs.msg import Float64, Int32
import numpy as np
from numpy import linalg
from mavros.srv import SetMode


#Constants
#*************************************
NODE_NAME='Blender'
#*************************************

class Blender():
  """The blender is used for blending accelerations calculated by different controllers and the
  obstacle avoidance controller. It also converts these accelerations to control outputs and publishes these on the topic irisX/mavros/rc/override.
  The blender also obtains data from the motion capture system, which it forwards to the controller in use."""
  

  def __init__(self):
    self.obstacle_avoidance = utils.Get_Parameter("obstacle_avoidance","False")
    self.avoidance = AvoidanceController()
    body_id = utils.Get_Parameter('body_id',8)
    self.load_id = utils.Get_Parameter('load_id',body_id)
    self.pitch_pub = rospy.Publisher("visualization/pitch",Float64,queue_size=10)
    self.roll_pub = rospy.Publisher("visualization/roll",Float64,queue_size=10)
    # Check what controller should be used
    self.controller_type = utils.Get_Parameter("controller_type","PID")
    if self.controller_type == "load_transport":
      print("LOAD")
      self.controller = LoadTransportController()
    else:
      print("PID")
      self.controller = PID()
    rospy.init_node(NODE_NAME)
    rospy.Service('blender/update_parameters', Empty, self.update_parameters)
    self.channel6 = 0
    self.instr = Instruction()
    self.load_parameters()
    rospy.Service('blender/set_channel6', SetChannel6, self.set_channel6)

  #Sets channel 6 to value
  ##@param channel6: the message sent by calling the service
  ##@return returns true if it was successfull 
  def set_channel6(self,channel6):
    self.channel6 = channel6.value
    return True
    
    
  # Gets target points and current points 
  ##@param target_point: the target point of the quad
  ##@param current_point: the current position of the quad 
  def init_subscriptions(self, target_point,current_point):
    #Subscribe to /derivator/pos_data to get position, velocity and acceleration
    rospy.Subscriber('security_guard/data_forward',QuadPositionDerived,self.new_point,current_point)
    #Subscribe to /security_guard/controller to get permission to publish to rc/override
    rospy.Subscriber('security_guard/controller',Int32,self.get_permission)

    
    if self.controller_type == "load_transport":
      #Subcribe to /trajectroy_gen/target to get target position, velocity and acceleration
      rospy.Subscriber('trajectory_gen/target_ext',QuadPositionDerivedExt,self.new_point,target_point)
    else:
      #Subcribe to /trajectroy_gen/target to get target position, velocity and acceleration
      rospy.Subscriber('trajectory_gen/target',QuadPositionDerived,self.new_point,target_point)

  # For load, if any
  ##@param current_load_point: the current position of the load
  def init_load_subscription(self, current_load_point):
    rospy.Subscriber("/body_data/id_"+str(self.load_id),QuadPositionDerived,self.new_point,current_load_point)

  def get_permission(self,data):
    self.instr.permission = data.data
    if not self.instr.start:
      if self.instr.permission == 0:
        self.instr.start=True

  def wait_for_security_guard(self,obj):
    """This function is called when starting to wait for the security guard to give permission
    to the blender to publish on the mavros/rc/override topic. This assures that everything is fine
    when starting an experiment."""
    rate=rospy.Rate(30)
    rospy.loginfo('Waiting for security guard ...')
    while not obj.start:
      if rospy.is_shutdown():
        return data_initdata_initdata_init
      rate.sleep()

  ##starts the blender
  def run_blender(self):
    """The function to be called to start the blender. It takes care of the calculations of control outputs
    and publishes these on the mavros/rc/override topic."""
    loop_rate=rospy.Rate(self.FREQUENCY)
    current_point=Point()
    current_load_point=Point()
    if self.controller_type == "load_transport":
      target_point=PointExt()
    else:
      target_point=Point()
    # Publish to RC Override
    rc_override=rospy.Publisher('mavros/rc/override',OverrideRCIn,queue_size=10)
    if type(self.controller) is PID:
      d_pub = rospy.Publisher('theD',Float64,queue_size=10)

    self.init_subscriptions(target_point, current_point)

    # For load
    self.init_load_subscription(current_load_point)

    data_init=OverrideRCIn()
    command=[self.CONTROL_NEUTRAL,self.CONTROL_NEUTRAL,self.CONTROL_ARMING_MIN,self.CONTROL_NEUTRAL,0,0,0,0]
    data_init.channels=command
    rc_override.publish(data_init)
    # Wait until the security guard is online
    self.wait_for_security_guard(self.instr)

    # Controller reset. For PID this means integral term initialized to 0.
    self.controller.reset()

    # Main loop
    while not rospy.is_shutdown():
      if not target_point.first_point_received:
        self.wait_for_first_point(target_point,rc_override,data_init,loop_rate)
        # Controller is reset. For the PID this means reinitialization of integral term.
        self.controller.reset()

      x,x_vel,x_acc=get_pos_vel_acc(current_point)  
      x_target,x_vel_target,x_acc_target=get_pos_vel_acc(target_point) 


      #current_load_point = deepcopy(current_point)
      #current_load_point.z = current_point.z - 5.0
      #If OK from security guard, publish the messages via Mavros to the drone
      if self.instr.permission == 0:
        if type(self.controller) is PID:
          u_cont = self.controller.get_output(current_point,target_point)
          u_cont[2] = u_cont[2] + 9.8
        elif type(self.controller) is LoadTransportController:
          u_cont = self.controller.get_output(current_load_point,current_point,target_point)

        u = self.blend(u_cont, current_point, target_point)
        command_controlled = self.get_controloutput(u,x,x_target)
  
        data=OverrideRCIn()
        data.channels=command_controlled
        rc_override.publish(data)
        ## Show the integrale action for the z component
        if type(self.controller) is PID:
          d_pub.publish(self.controller.get_d_updated()[1])
      elif self.instr.permission == 2:
        target_point.x = current_point.x
        target_point.y = current_point.y
        target_point.z = current_point.z
        target_point.yaw = current_point.yaw
        target_point.x_vel = 0  
        target_point.y_vel = 0
        target_point.z_vel = 0
        target_point.yaw_vel = 0      
        target_point.x_acc = 0
        target_point.y_acc = 0
        target_point.z_acc = 0
        target_point.yaw_acc = 0
        self.landmode()
      else:
        break 
      loop_rate.sleep()
                

  # From the required acceleration u, the control outputs are calculated: roll, pitch,
  # throttle and yaw_rate
  ##@param u: the accelerations calculated from blending the accelerations calculated by the controllers
  ##@param x: the current position
  ##@param x_target: the target position
  ##@return the control output that is send to the drone
  def get_controloutput(self,u,x,x_target):
    AUX=np.array([0.]*3)
    AUX_rot=np.array([0.]*3)
    AUX[0] = u[0]
    AUX[1] = u[1]
    AUX[2] = u[2]


    #take into consideration the yaw angle
    AUX_rot[0] = math.cos(math.radians(-x[3]))*AUX[0]-math.sin(math.radians(-x[3]))*AUX[1]
    AUX_rot[1] = math.sin(math.radians(-x[3]))*AUX[0]+math.cos(math.radians(-x[3]))*AUX[1]
    AUX_rot[2] = AUX[2]

    # This is a complement to the saturation limit below, and eliminates the
    # risk that norm_AUX becomes too large when the angle between AUX_rot and
    # the z axis is too large for the quad to set out. (This would make the
    # quad fly off in the z direction.)
    angle_limit = 5*math.pi/180
    if abs(math.atan(AUX_rot[0]/AUX_rot[2])) > angle_limit:
      AUX_rot[0] = np.sign(AUX_rot[0])*math.tan(angle_limit)*abs(AUX_rot[2])
    if abs(math.atan(AUX_rot[1]/AUX_rot[2])) > angle_limit:
      AUX_rot[1] = np.sign(AUX_rot[1])*math.tan(angle_limit)*abs(AUX_rot[2])

    norm_AUX=linalg.norm(AUX_rot)

    #yaw control:
    diff=self.angular_difference(x[3],x_target[3])
    w_yaw=-self.K_yaw*(math.radians(diff))

    #set values:
    throttle=(self.CONTROL_CANCEL_GRAVITY)*math.sqrt(norm_AUX/9.8)
    yaw_rate=self.CONTROL_NEUTRAL - self.N_yaw*self.saturation(w_yaw/self.w_inf,-1,1)
    pitch=self.CONTROL_NEUTRAL-self.Ktt*math.asin(AUX_rot[0]/norm_AUX)
    roll=self.CONTROL_NEUTRAL-self.Kphi*math.asin(AUX_rot[1]/norm_AUX)


    # Implement some saturation
    throttle=self.saturation(throttle,1000,2000)
    pitch=self.saturation(pitch,1200,1800)
    roll=self.saturation(roll,1200,1800)
    self.pitch_pub.publish(pitch)
    self.roll_pub.publish(roll)
    return [roll,pitch,throttle,yaw_rate,0,self.channel6,0,0]


  # Read the outputs of the controller and collision avoidance, then "blends"
  # the outputs  
  ##@param u_cont: the acceleration calculated by the controller
  ##@param current_point: the current position of the quad
  ##@param target_point: the target position of the quad
  ##@return the acceleration after blending obstacle avoidance and the acceleration from the controller
  def blend(self,u_cont,current_point,target_point):
    u = np.array([0.,0.,0.])
    u_obst = self.avoidance.get_potential_output()
    if self.obstacle_avoidance:
      alpha = self.avoidance.get_blending_constant()
      utils.loginfo(alpha)
    else:
      alpha = 0
    for i in range(0,2):
      u[i] = alpha * u_obst[i] + (1-alpha) * u_cont[i]
    u[2] = u_cont[2] 
    return u
 
  ##@param data: data from the motion capture system
  ##@param point_obj: a point to be updated 
  def new_point(self,data,point_obj):
    """Checks if the first point from the trajectory generation has been
    received. If not it notifies that the first point now has been received.
    The udpdate function of the point is called to update the point to be tracked.
    This is the callback function of the subscriptions to all topics that publish
    points (QuadPositionDerived) to the blender."""
    if not point_obj.first_point_received:
      point_obj.first_point_received=True

    point_obj.update_point(data)

  ##@param value: a real number
  ##@param minimum: the minimum allowed value of value
  ##@param maximum: the maximum allowed value of value
  ##@return: value if minimum <= value <= maximum, minimum if value < minimum < maximum, maximum if minimum < maximum < value
  def saturation(self,value,minimum,maximum):
    value=max(minimum,min(maximum,value))
    return value

  ##@param target_obj: the target point
  ##@param channel: the publisher that publishes to the desired topic
  ##@param data: the message to be published on channel
  ##@param rate: the publishing rate
  def wait_for_first_point(self,target_obj,channel,data,rate):
    rospy.loginfo('Waiting for first point ...')
    while not target_obj.first_point_received:
      #publish low value on the throttle channel, so the drone does not disarm while waiting
      channel.publish(data)
      rate.sleep()

    rospy.loginfo('First point received')      

  ##@param current_angle: an angle given in degrees
  ##@param target_angle: an angle given in degrees
  ##@return: the difference between current_angle and target_angle in the interval [-180,180]
  def angular_difference(self,current_angle,target_angle):
    ang_diff=current_angle-target_angle

    if math.fabs(ang_diff)>180:
      if ang_diff>0:
        ang_diff=ang_diff-360
      else:
        ang_diff=ang_diff+360

    return ang_diff
  
  ##@param msg: the message produced when calling the service blender/update_parameters   
  def update_parameters(self,msg):
    """This function is the function called when the service blender/update_parameters is called.
    A notification of the update is given and the function that loads the parameters from the parameter
    server is given."""
    utils.loginfo('Blender parameters loaded')
    self.load_parameters()

    # If controller type is changed
    if self.controller_type == "load_transport" and type(self.controller) != LoadTransportController:
      self.controller = LoadTransportController()
    elif self.controller_type == "PID" and type(self.controller) != PID:
      self.controller = PID()

    return []
  
  # Read parameters for Blender
  def load_parameters(self):
    """This function loads the parameters necessary for the blender from the parameter server."""		
    self.N_yaw = utils.Get_Parameter("N_yaw",500)
    self.K_yaw = utils.Get_Parameter("K_yaw",2)
    self.w_inf = utils.Get_Parameter("w_inf",5)
    self.Ktt = utils.Get_Parameter("Ktt",1000)/(20*math.pi/180)
    self.Kphi = utils.Get_Parameter("Kphi",1000)/(20*math.pi/180)
    self.CONTROL_MIN = utils.Get_Parameter("CONTROL_MIN",1000)
    self.CONTROL_NEUTRAL = utils.Get_Parameter("CONTROL_NEUTRAL",1500)
    self.CONTROL_MAX = utils.Get_Parameter("CONTROL_MAX",2000)
    self.CONTROL_ARMING_MIN = utils.Get_Parameter("CONTROL_ARMING_MIN",1025)
    self.CONTROL_CANCEL_GRAVITY = utils.Get_Parameter("CONTROL_CANCEL_GRAVITY",1400)	

    self.FREQUENCY = utils.Get_Parameter("CONTROLLER_FREQUENCY",30)

    self.controller_type = utils.Get_Parameter("controller_type","PID")


  ## Switch the quad to the landing mode
  def landmode(self):
    """This function uses the service mavros/set_mode to set the mode of the quad to the 
    landing mode."""
    mode_set=False
    for i in range(0,10):
        if not mode_set:
            try:
                set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
                mode_set = set_mode(base_mode=0, custom_mode='LAND')
            except rospy.ServiceException as ex:
                rospy.logerr('[LD]: set_mode attempt not successful')

if __name__ == "__main__":
  bl = Blender()
  bl.run_blender()

#EOF
