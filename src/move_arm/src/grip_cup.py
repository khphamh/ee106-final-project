#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
import rospy
import random
from intera_interface import gripper as robot_gripper

minX = 0.781
maxX = 0.749 
minZ = -0.118
maxZ = 1.204
maxY = 0.620
minY = 0.572
Z_diff = maxZ - minZ

def main():
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query2')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    while not rospy.is_shutdown():
        input('Press [ Enter ]: ')
        
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = "right_gripper_tip"

        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        # Set the desired orientation for the end effector HERE
        # Initial Position in picture 1 of google doc
        request.ik_request.pose_stamped.pose.position.x = 0.645
        request.ik_request.pose_stamped.pose.position.y = 0.076
        request.ik_request.pose_stamped.pose.position.z = -0.095        
        request.ik_request.pose_stamped.pose.orientation.x = 0.534
        request.ik_request.pose_stamped.pose.orientation.y = 0.526
        request.ik_request.pose_stamped.pose.orientation.z = -0.434
        request.ik_request.pose_stamped.pose.orientation.w = 0.0



        #rospy.init_node('gripper_test')

        # Set up the right gripper
        right_gripper = robot_gripper.Gripper('right_gripper')

        # Calibrate the gripper (other commands won't work unless you do this first)
        print('Calibrating...')
        right_gripper.calibrate()
        rospy.sleep(2.0)

        
        
        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            ##group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK and execute
            group.go()

            # Close the right gripper
            print('Closing...')
            right_gripper.close()
            rospy.sleep(3.0)

            request.ik_request.pose_stamped.pose.position.x = minX
            request.ik_request.pose_stamped.pose.position.y = random.uniform(minY, maxY)
            request.ik_request.pose_stamped.pose.position.z = random.uniform(minZ + random.uniform(0.5 * Z_diff, Z_diff), maxZ)

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # Plan IK and execute
            group.go()

            # Open the right gripper
            print('Opening...')
            right_gripper.open()
            rospy.sleep(3.0)
            print('Done!')
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

# Python's syntax for a main() method
if __name__ == '__main__':
    main()