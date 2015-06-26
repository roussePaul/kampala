#!/usr/bin/env python
import rospy

# script with useful function 
  

def logerr(msg):
  rospy.logerr(get_message(msg))

def logwarn(msg):
  rospy.logerr(get_message(msg))

def loginfo(msg):
  rospy.logerr(get_message(msg))

def get_message(msg):
  return '\033[92m\033[1m['+rospy.get_name()+']\033[0m ' + msg 