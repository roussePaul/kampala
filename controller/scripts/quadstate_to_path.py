#!/usr/bin/env python
import rospy
from mocap.msg import QuadPositionDerived
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, PoseStamped, Quaternion, Vector3, Quaternion
import tf
import math

# This scripts listens to planned trajectory paths and enables a
# visualization of them by converting them to Path msgs. It also
# listens the the actual path of the robot and therefore enables a
# visualization of the error.


def listener():
    rospy.init_node('visualizer', anonymous=True)
    rospy.Subscriber('trajectory_gen/target',QuadPositionDerived, planned_path_converter)
    rospy.Subscriber('security_guard/data_forward',QuadPositionDerived, actual_path_converter) # change?
    rospy.spin()

# Publishes planned path
def planned_path_converter(quadstate):
    quat = tf.transformations.quaternion_from_euler(math.radians(quadstate.roll), math.radians(quadstate.pitch), math.radians(quadstate.yaw))
    pose = Pose(Point(quadstate.x, quadstate.y, quadstate.z), Quaternion(*quat))
    pose_stamped = PoseStamped(Header(0, rospy.Time.now(), "/map"), pose)
    planned_path.poses.append(pose_stamped)
    pub_planned.publish(planned_path)

    pub_planned_marker.publish(pose_stamped)

# Publishes actual path
def actual_path_converter(quadstate):
    quat = tf.transformations.quaternion_from_euler(math.radians(quadstate.roll), math.radians(quadstate.pitch), math.radians(quadstate.yaw))
    pose = Pose(Point(quadstate.x, quadstate.y, quadstate.z), Quaternion(*quat))
    pose_stamped = PoseStamped(Header(0, rospy.Time.now(), "/map"), pose)
    actual_path.poses.append(pose_stamped)
    pub_actual.publish(actual_path)
    
    pub_actual_marker.publish(pose_stamped)

if __name__ == '__main__':
    planned_path = Path()
    planned_path.header.frame_id = "/map"
    actual_path = Path()
    actual_path.header.frame_id = "/map"
    pub_planned = rospy.Publisher('visualizer/planned_path',Path, queue_size=10)
    pub_actual = rospy.Publisher('visualizer/actual_path',Path, queue_size=10)
    pub_actual_marker = rospy.Publisher('visualizer/actual_marker',PoseStamped, queue_size=10)
    pub_planned_marker = rospy.Publisher('visualizer/planned_marker',PoseStamped, queue_size=10)
    listener()
