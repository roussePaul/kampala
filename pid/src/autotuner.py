#!/usr/bin/env python

import rospy

class Autotuner:
	controller_list_param = "/autotuner/controllers"

	@staticmethod
	def get_controller_list():
		if rospy.has_param(Autotuner.controller_list_param):
			return rospy.get_param(Autotuner.controller_list_param)
		else:
			return []

	@staticmethod
	def add_controller(node_name):
		node_list = Autotuner.get_controller_list()
		node_name = rospy.get_name()+"/"+node_name
		node_list.append(node_name)
		rospy.set_param(Autotuner.controller_list_param,node_list)

	@staticmethod
	def remove_controller(node_name):
		node_list = Autotuner.get_controller_list()
		node_name = rospy.get_name()+"/"+node_name
		if node_name in node_list:
			node_list.remove(node_name)
		rospy.set_param(Autotuner.controller_list_param,node_list)





