#!/usr/bin/env python

## Publishes messages that will be used by rqt_plot nodes to show the errors between target and actual position/velocity

import rospy
from mocap.msg import QuadPositionDerived
from controller.msg import PlotData


class Point():
	def __init__(self):
		self.x=0
		self.y=0
		self.z=0
		self.yaw=0
		self.x_vel=0
		self.y_vel=0
		self.z_vel=0
		self.yaw_vel=0
		self.x_acc=0
		self.y_acc=0
		self.z_acc=0
		self.yaw_acc=0


def New_Point(data,obj):
	obj.x=data.x
	obj.y=data.y
	obj.z=data.z
	obj.yaw=data.yaw
	obj.x_vel=data.x_vel
	obj.y_vel=data.y_vel
	obj.z_vel=data.z_vel
	obj.yaw_vel=data.yaw_vel
	obj.x_acc=data.x_acc
	obj.y_acc=data.y_acc
	obj.z_acc=data.z_acc
	obj.yaw_acc=data.yaw_acc
	return


if __name__=='__main__':
	rospy.init_node('error_vis')
	loop_rate=rospy.Rate(30)

	current_point=Point()
	target_point=Point()

	rospy.Subscriber('security_guard/data_forward',QuadPositionDerived,New_Point,current_point)
	rospy.Subscriber('trajectory_gen/target',QuadPositionDerived,New_Point,target_point)

	x_coord=rospy.Publisher('error_vis/x_coord',PlotData,queue_size=10)
	x_velocity=rospy.Publisher('error_vis/x_velocity',PlotData,queue_size=10)
	y_coord=rospy.Publisher('error_vis/y_coord',PlotData,queue_size=10)
	y_velocity=rospy.Publisher('error_vis/y_velocity',PlotData,queue_size=10)
	z_coord=rospy.Publisher('error_vis/z_coord',PlotData,queue_size=10)
	z_velocity=rospy.Publisher('error_vis/z_velocity',PlotData,queue_size=10)

	while not rospy.is_shutdown():
		x_pos=PlotData(target_point.x,current_point.x)
		x_vel=PlotData(target_point.x_vel,current_point.x_vel)
		y_pos=PlotData(target_point.y,current_point.y)
		y_vel=PlotData(target_point.y_vel,current_point.y_vel)
		z_pos=PlotData(target_point.z,current_point.z)
		z_vel=PlotData(target_point.z_vel,current_point.z_vel)

		x_coord.publish(x_pos)
		x_velocity.publish(x_vel)
		y_coord.publish(y_pos)
		y_velocity.publish(y_vel)
		z_coord.publish(z_pos)
		z_velocity.publish(z_vel)

		loop_rate.sleep()