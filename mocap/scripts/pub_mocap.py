#!/usr/bin/env python
#Vinzenz Minnig, 2015

#imports the data from the Qualysis motion capture system into the ROS space
#data has to be requested by a service that this node provides
#the different services are:
#   get_available_bodies()      ->  returns a list of avalable bodies by their id
#   get_data(body_id)           ->  returns all the data from a specific body (body id passed as an argument)
#                                   the returned data is in the service format BodyData, which contains
#                                   x,y,z,pitch,roll,yaw

import sys
import rospy
import numpy as np
import tf

from mocap.msg import QuadPosition
from mocap.srv import Bodies
from mocap.srv import BodyData
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
import std_msgs.msg


class PubMocap:
	def __init__(self):
		self.frequency = rospy.get_param('frequency',100)
		self.track_id = rospy.get_param('mocap_track',[1])
		self.track_ns = rospy.get_param('mocap_ns',['iris'])

		cov_pos = 0.001
		cov_angle = 0.01
		P = np.mat(np.diag(np.concatenate( ([cov_pos]*3,[cov_angle]*3) )))
		covariance = P.ravel().tolist()[0]
		self.mocap_covariance = rospy.get_param('mocap_covariance',covariance)

		self.data = []

		self.start_service_call()

		self.pub = {}
		for (i,ns) in zip(self.track_id,self.track_ns):
			self.pub[i] = rospy.Publisher('/'+ns+'/mocap_output', PoseWithCovarianceStamped, queue_size=10)

		r = rospy.Rate(self.frequency) 
		while not rospy.is_shutdown():
			self.publish()
			r.sleep()

	def publish(self):
		for (i,ns) in zip(self.track_id,self.track_ns):
			pose = self.get_data_body(i)
			try:
				self.pub[i].publish(pose)
			except rospy.ServiceException as exc:
				print("Service did not process request: " + str(exc))

	def get_data_body(self, body_id):
		try:
			resp1 = self.call_body_data(body_id)
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))

		msg = PoseWithCovarianceStamped()
		pose = Pose()
		pose.position.x = resp1.pos.x
		pose.position.y = resp1.pos.y
		pose.position.z = resp1.pos.z

		quaternion = tf.transformations.quaternion_from_euler(resp1.pos.roll, resp1.pos.pitch, resp1.pos.yaw)
		#type(pose) = geometry_msgs.msg.Pose
		pose.orientation.x = quaternion[0]
		pose.orientation.y = quaternion[1]
		pose.orientation.z = quaternion[2]
		pose.orientation.w = quaternion[3]
		
		h = std_msgs.msg.Header()
		h.stamp = rospy.Time.now()
		msg.header = h
		msg.header.frame_id= 'world'
		msg.pose.covariance = self.mocap_covariance
		msg.pose.pose = pose

		return msg

	def start_service_call(self):
		rospy.wait_for_service('/mocap_get_data')
		self.call_body_data = rospy.ServiceProxy('mocap_get_data', BodyData)

	def start_pub(self):
		rospy.Subscriber("/gazebo/model_states",ModelStates,self.update_positions)

if __name__=="__main__":
	rospy.init_node("pub_mocap")
	pm = PubMocap()

#EOF
