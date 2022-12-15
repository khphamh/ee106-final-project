#!/usr/bin/env python
import rospy

from intera_interface import gripper as robot_gripper

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

def main():
	rospy.init_node('grip')

	# Set up the right gripper
	right_gripper = robot_gripper.Gripper('right_gripper')

	# Calibrate the gripper (other commands won't work unless you do this first)
	# print('Calibrating...')
	# right_gripper.calibrate()
	# rospy.sleep(0.5)

	# Open the right gripper
	print('Opening...')
	right_gripper.open()
	rospy.sleep(3)
	print('Done!')


if __name__ == '__main__':
	main()