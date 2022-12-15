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


	# Close the right gripper
	print('Closing...')
	right_gripper.close()
	rospy.sleep(0.5)


if __name__ == '__main__':
	main()