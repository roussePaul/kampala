#!/usr/bin/env python
import rospy

# script with useful function 
  

def logerr(msg):
  rospy.logerr(get_header()+'\033[91m'+msg+'\033[0m')

def logwarn(msg):
  rospy.logwarn(get_header()+'\033[93m'+msg+'\033[0m')

def loginfo(msg):
  rospy.loginfo(get_header()+'\033[0m'+msg)

def get_header():
  return '\033[92m\033[1m\t['+rospy.get_name()+'] ' 
