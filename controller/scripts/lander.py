#!/usr/bin/env python
import rospy
import mavros
from mavros.srv import SetMode
from mavros.utils import *
from controller.msg import Permission
from mavros.msg import OverrideRCIn

"""This script contains the functions necessary to land the drone given permission by the 
security guard."""

def callback(data):
    """This is the callback function of the subscription to the topic security_guard/lander.
    If permission is given the landing mode is set and neutral commands are published on the
    mavros/rc/override topic."""
    if data.permission==True:
        landmode()
        rospy.sleep(0.5)
        landpub()

## Switch the quad to the landing mode
def landmode():
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

## Start to publish the null command on the RC topic (all the stick in the neutral position).
## This function publish until ROS is shutdown.
def landpub():
    """This function publishes neutral commands on mavros/rc/override."""
    pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=1000)
    r = rospy.Rate(30)
    all_sticks_centered = [1500,1500,1000,1500,0,0,0,0]
    while not rospy.is_shutdown():
        pub.publish(all_sticks_centered)
        r.sleep()
    
## Init the lander node.
def lander():
    """This function initializes the lander node and subscribes to the topic security_guard/lander."""
    rospy.init_node('Lander')

    rospy.Subscriber("security_guard/lander", Permission, callback)

    rospy.spin()

if __name__ == '__main__':
    lander()
