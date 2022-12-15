#!/usr/bin/env python
import rospy

from intera_interface import gripper as robot_gripper
import argparse
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped
import PyKDL
from tf_conversions import posemath
from intera_interface import Limb

#def main():
import rospy

from intera_interface import gripper as robot_gripper

rospy.init_node('gripper_test')

# Set up the right gripper
right_gripper = robot_gripper.Gripper('right_gripper')

# Calibrate the gripper (other commands won't work unless you do this first)
print('Calibrating...')
right_gripper.calibrate()
rospy.sleep(2.0)

# Close the right gripper
print('Closing...')
right_gripper.close()
rospy.sleep(1.0)

# Open the right gripper
print('Opening...')
right_gripper.open()
rospy.sleep(1.0)
print('Done!')