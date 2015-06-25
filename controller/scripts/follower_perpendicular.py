#!/usr/bin/env python
import rospy
from math import sqrt
from numpy.linalg import norm
from numpy import dot, cross
from mocap.msg import QuadPositionDerived

# This script generates points for a follower following a leader,
# given the id of the leader and a constant distance between the
# two, the offset. The follower will be positioned so that their
# velocities are parallel, but the distance vector between them are
# perpendicular to their velocity, in the plane of the normal vector.


def starter():
  # Getting parameters
  offset = rospy.get_param("trajectory_generator/offset",1)
  leader_id = rospy.get_param("trajectory_generator/leader_id",0) # change?
  pub = rospy.Publisher('trajectory_gen/target',QuadPositionDerived, queue_size=10)
  
  # Initial value of leader state
  leader_state = QuadPositionDerived()
  
  # Publication
  rospy.init_node('follower',anonymous=True)
  r = 15 # Change rate to desired value
  rate = rospy.Rate(r)
  while not rospy.is_shutdown():
    leader_state = getLeaderState(leader_id, leader_state)
    follower_state = calculateState(leader_state, offset)
    pub.publish(follower_state)
    rate.sleep()


# Gets the position, velocity and acceleration of the leader
def getLeaderState(leader_id, leader_state):
  rospy.wait_for_service('derivator')
  try:
    derivator = rospy.ServiceProxy('derivator', QuadPositionDerived)

    # Old values
    leader_state_simple = QuadPositionDerived(leader_state.x, leader_state.y, leader_state.z, leader_state.yaw)
    leader_vel_state_simple = QuadPositionDerived(leader_state.x_vel, leader_state.y_vel, leader_state.z_vel, 0)
    
    # New values
    leader_state = derivator(leader_id, leader_state_simple, leader_vel_state_simple)
    return leader_state
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e


# Calculates the state of the follower from the leader state
def calculateState(leader_state, offset):
  follower_state = QuadState()
  leader_pos = [leader_state.x, leader_state.y, leader_state.z]
  leader_vel = [leader_state.x_vel, leader_state.y_vel, leader_state.z_vel]
  leader_acc = [leader_state.x_acc, leader_state.y_acc, leader_state.z_acc]

  # Curvature
  if norm(cross(leader_vel,leader_acc)) != 0:
    rho = norm(leader_vel)**3/norm(cross(leader_vel,leader_acc))
  else:
    rho = 'inf'

  # Direction of tangenetial coordinate
  e_t = leader_vel/norm(leader_vel)

  # Direction of normal coordinate
  # Linear motion is a special case. We assume linear motion if rho >
  # 20 m. Also, we ignore too small rho, and take those as measurement
  # errors. In this case, follower continues forward
  if rho == 'inf' or rho > 20 or rho < 0.05:
    d = follower_pos - leader_pos
    e_n = ((follower_pos - leader_pos) - dot((follower_pos - leader_pos), e_t))/norm((follower_pos - leader_pos) - dot((follower_pos - leader_pos), e_t))
  else:
    e_n = (leader_acc - dot(leader_acc,e_t)*e_t)/norm(leader_acc - dot(leader_acc,e_t)*e_t)

  # Position of the follower
  follower_pos = leader_pos + offset*e_n

  # Velocity of follower
  if rho != 'inf':
    follower_vel = (rho + offset)/rho*leader_vel
  else:
    follower_vel = leader_vel

  # Acceleration of follower
  if rho != 'inf':
    follower_acc = (rho + offset)/rho*dot(leader_acc,e_t)*e_t + norm(follower_vel)**2/(rho + offset)*e_n
    # (Notice the two contributions to the acceleration)
  else:
    follower_acc = leader_acc

  follower_state.x = follower_pos[0]
  follower_state.y = follower_pos[1]
  follower_state.z = follower_pos[2]

  follower_state.x_vel = follower_vel[0]
  follower_state.y_vel = follower_vel[1]
  follower_state.z_vel = follower_vel[2]

  follower_state.x_vel = follower_acc[0]
  follower_state.y_vel = follower_acc[1]
  follower_state.z_vel = follower_acc[2]

  follower_state.yaw = leader_state.yaw
  follower_state.yaw_vel = leader_state.yaw_vel
  follower_state.yaw_acc = leader_state.yaw_acc

  return follower_state


if __name__ == '__main__':
  try:
    starter()
  except rospy.ROSInterruptException:
    pass
