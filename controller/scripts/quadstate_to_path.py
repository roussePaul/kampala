#!/usr/bin/env python

"""This scripts listens to planned trajectory paths and enables a
visualization of them by converting them to Path msgs. It also
listens the the actual path of the robot and therefore enables a
visualization of the error."""


import rospy
from mocap.msg import QuadPositionDerived
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, PoseStamped, Quaternion, Vector3, Quaternion
import tf
import math


def listener():
    """This function initializes the visualizer node and subscribes to the topics trajectory_gen/target and security_guard/data_forward,
    which are relevant for the visualization of the error."""
    rospy.init_node('visualizer')
    rospy.Subscriber('trajectory_gen/target',QuadPositionDerived, planned_path_converter)
    rospy.Subscriber('security_guard/data_forward',QuadPositionDerived, actual_path_converter) 
    rospy.spin()

##@param quadstate: the state of the quad published on the topic trajectory_gen/target
def planned_path_converter(quadstate):
    """This is the callback function of the subscriber to the topic trajectory_gen/target. The state of the quad is transformed into a quaternion
    and added to a path. The path is published to rviz for visualization on the topic visualizer/planned_path. To prevent the visualization from
    freezing the list containing the path cannot be longer than a certain length. If it is of the maximum length and a new quaternion is added
    an old one is removed. This means that old parts of the path are starting to vanish after some time."""
    quat = tf.transformations.quaternion_from_euler(math.radians(quadstate.roll), math.radians(quadstate.pitch), math.radians(quadstate.yaw))
    pose = Pose(Point(quadstate.x, quadstate.y, quadstate.z), Quaternion(*quat))
    pose_stamped = PoseStamped(Header(0, rospy.Time.now(), "/map"), pose)
    planned_path.poses.append(pose_stamped)
    if len(planned_path.poses) > max_length:
        planned_path.poses.pop(0)

    pub_planned.publish(planned_path)
    pub_planned_marker.publish(pose_stamped)

# Publishes actual path
def actual_path_converter(quadstate):
    """This is the equivalent of planned_path_converter for the actual path. It is the callback function for the subscription to security_guard/data_forward
    and publishes on visualizer/actual_path."""
    quat = tf.transformations.quaternion_from_euler(math.radians(quadstate.roll), math.radians(quadstate.pitch), math.radians(quadstate.yaw))
    pose = Pose(Point(quadstate.x, quadstate.y, quadstate.z), Quaternion(*quat))
    pose_stamped = PoseStamped(Header(0, rospy.Time.now(), "/map"), pose)
    actual_path.poses.append(pose_stamped)
    if len(actual_path.poses) > max_length:
        actual_path.poses.pop(0)
    
    pub_actual.publish(actual_path)
    pub_actual_marker.publish(pose_stamped)

if __name__ == '__main__':
    planned_path = Path()
    planned_path.header.frame_id = "/map"
    actual_path = Path()
    actual_path.header.frame_id = "/map"
    max_length = 1000 # Max length of path vector
    pub_planned = rospy.Publisher('visualizer/planned_path',Path, queue_size=10)
    pub_actual = rospy.Publisher('visualizer/actual_path',Path, queue_size=10)
    pub_actual_marker = rospy.Publisher('visualizer/actual_marker',PoseStamped, queue_size=10)
    pub_planned_marker = rospy.Publisher('visualizer/planned_marker',PoseStamped, queue_size=10)
    listener()
