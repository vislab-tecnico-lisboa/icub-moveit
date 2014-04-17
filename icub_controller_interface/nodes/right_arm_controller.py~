#! /usr/bin/env python

import roslib; roslib.load_manifest('icub_controllers')
import rospy
import actionlib

from control_msgs.msg import *


class FollowJointTrajectoryServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('right_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction, self.execute, False)
    self.server.start()

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    print "EXECUTED"
    self.server.set_succeeded()


if __name__ == '__main__':
  rospy.init_node('right_arm_controller')
  server = FollowJointTrajectoryServer()
  rospy.spin()
