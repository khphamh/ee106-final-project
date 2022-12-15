#!/usr/bin/env python
import os
import numpy as np
import cv2
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
import rospy
import intera_interface
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
import time
from skimage import io
IMG_DIR = r'/home/cc/ee106a/fa22/class/ee106a-acn/ros_workspaces/final_project/src/move_arm/img'

initial_translation = [0.680, 0.225, 0.148] 
initial_quaternion = [0.003, 0.705, 0.002, 0.710]

def move(translation, quaternion, initialize=True):
    """
    Move the robot arm to the specified configuration.
    Call using:
    $ rosrun intera_examples go_to_cartesian_pose.py  [arguments: see below]

    -p 0.4 -0.3 0.18 -o 0.0 1.0 0.0 0.0 -t right_hand
    --> Go to position: x=0.4, y=-0.3, z=0.18 meters
    --> with quaternion orientation (0, 1, 0, 0) and tip name right_hand
    --> The current position or orientation will be used if only one is provided.

    -q 0.0 -0.9 0.0 1.8 0.0 -0.9 0.0
    --> Go to joint angles: 0.0 -0.9 0.0 1.8 0.0 -0.9 0.0 using default settings
    --> If a Cartesian pose is not provided, Forward kinematics will be used
    --> If a Cartesian pose is provided, the joint angles will be used to bias the nullspace

    -R 0.01 0.02 0.03 0.1 0.2 0.3 -T
    -> Jog arm with Relative Pose (in tip frame)
    -> x=0.01, y=0.02, z=0.03 meters, roll=0.1, pitch=0.2, yaw=0.3 radians
    -> The fixed position and orientation paramters will be ignored if provided

    """
    try:
        if initialize:
            rospy.init_node('go_to_cartesian_pose_py')
        limb = Limb()

        traj_options = TrajectoryOptions()
        traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
        traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)

        wpt_opts = MotionWaypointOptions(max_linear_speed=0.3,
                                         max_linear_accel=0.3,
                                         max_rotational_speed=1.0,
                                         max_rotational_accel=1.0,
                                         max_joint_speed_ratio=1.0)
        waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

        joint_names = limb.joint_names()

        endpoint_state = limb.tip_state('right_hand')
        if endpoint_state is None:
            rospy.logerr('Endpoint state not found with tip name %s', 'right_hand')
            return None
        pose = endpoint_state.pose
        
        pose.position.x = translation[0]
        pose.position.y = translation[1]
        pose.position.z = translation[2]
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        poseStamped = PoseStamped()
        poseStamped.pose = pose


        joint_angles = limb.joint_ordered_angles()
        waypoint.set_cartesian_pose(poseStamped, 'right_hand', joint_angles)




        # rospy.loginfo('Sending waypoint: \n%s', waypoint.to_string())

        traj.append_waypoint(waypoint.to_msg())

        result = traj.send_trajectory(timeout=None)
        if result is None:
            rospy.logerr('Trajectory FAILED to send')
            return

        # if result.result:
        #     # rospy.loginfo('Motion controller successfully finished the trajectory!')
        # else:
        #     rospy.logerr('Motion controller failed to complete the trajectory with error %s',
        #                  result.errorId)
    except rospy.ROSInterruptException:
        rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')


def show_image_callback(img_data, xxx_todo_changeme):
    """The callback function to show image by using CvBridge and cv
    """
    (edge_detection, window_name) = xxx_todo_changeme
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")
    except CvBridgeError as err:
        print("show_image_call_back")
        rospy.logerr(err)
        return
    if edge_detection == True:
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        # customize the second and the third argument, minVal and maxVal
        # in function cv2.Canny if needed
        get_edge = cv2.Canny(blurred, 10, 100)
        cv_image = np.hstack([get_edge])
    edge_str = "(Edge Detection)" if edge_detection else ''
    directory = r'/home/cc/ee106a/fa22/class/ee106a-acn/ros_workspaces/final_project/src/move_arm/img'
    os.chdir(directory)    
    file_name = 'dice_image.jpg'

    cv2.imwrite(file_name, cv_image)
    cv2.waitKey(3)

def take_image(initialize=True):
    """Camera Display Example

    Cognex Hand Camera Ranges
        - exposure: [0.01-100]
        - gain: [0-255]
    Head Camera Ranges:
        - exposure: [0-100], -1 for auto-exposure
        - gain: [0-79], -1 for auto-gain
    """
    rp = intera_interface.RobotParams()
    valid_cameras = rp.get_camera_names()
    #print(valid_cameras)
    if not valid_cameras:
        rp.log_message(("Cannot detect any camera_config"
            " parameters on this robot. Exiting."), "ERROR")
        return
    

    #print("Initializing node... ")
    if initialize:
        rospy.init_node('camera_display', anonymous=True)
    cameras = intera_interface.Cameras()
    if not cameras.verify_camera_exists('right_hand_camera'):
        rospy.logerr("Could not detect the specified camera, exiting the example.")
        return
    # rospy.loginfo("Opening camera '{0}'...".format('right_hand_camera'))
    cameras.start_streaming('right_hand_camera')
    cameras.set_callback('right_hand_camera', show_image_callback,
        rectify_image=False, callback_args=(None, 'right_hand_camera'))

def clean_shutdown():
    #print("Shutting down camera_display node.")
    cv2.destroyAllWindows()

def calculate_error():
    image_location = IMG_DIR + '/dice_image.jpg'
        #_ = io.imread(image_location)
    with open(os.path.join(IMG_DIR, 'dice_image.jpg'), 'rb') as f:
        check_chars = f.read()[-2:]
    if check_chars != b'\xff\xd9':
        print("premature end")
        time.sleep(2)
        return (0, 0)
    image = cv2.imread(image_location)
    if type(image) == type(None):
        return (0, 0)
    print(image.shape)

    image_blurred = cv2.blur(image, (3,3))
    img_gray = cv2.cvtColor(image_blurred, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(img_gray, 140, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
                                        
    # draw contours on the original image
    #image_copy = image.copy()
    #cv2.drawContours(image=image_copy, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)


    min_dice_area = 1000
    max_dice_area = 8000
    dice_locations = []
    cnts = contours
    for i in range(len(contours)):
        rect = cv2.minAreaRect(cnts[i])
        box = np.int0(cv2.boxPoints(rect))
        #print(rect)
        #print(rect[1][0]*rect[1][1])
        '''if min_dot_area < rect[1][0]*rect[1][1] < max_dot_area:
            #print("dot", i, rect[1][0]*rect[1][1])
            cv2.drawContours(image_copy, [box], 0, (36,255,12), 3)
            dot_total += 1'''

        if min_dice_area < rect[1][0]*rect[1][1] < max_dice_area:
            dice_locations.append(rect)
            #cv2.drawContours(image_copy, [box], 0, (36,255,12), 3)
    #cv2.imwrite("temp_dice_pic.jpg", image_copy)
    # cv2.polylines(image, [box], True, (36,255,12), 3)
    print(dice_locations)
    if not dice_locations:
        return (0, 0)

    error = np.array( (np.shape(image)[1], np.shape(image)[0]) )/2 - np.array(dice_locations[0][0])
    print(error)
    return error


if  __name__ == '__main__':
    curr_translation = initial_translation
    curr_quaternion = initial_quaternion
    
    move(curr_translation,curr_quaternion)
    take_image(False)
    time.sleep(5)
    dx, dy = calculate_error()
    while (dx**2 > 10 or dy**2 > 10) or (dx == 0 and dy == 0):
        print("start big loop ----------------------------------------")
        while dx**2 > 10 or dx == 0:
            print(dx, dy, "----------------------------------------------------------------------------------------------------------------------------")
            curr_translation[1] += dx*(10**-4)
            move(curr_translation, curr_quaternion, False)
            take_image(False)
            dx, dy = calculate_error()
        print("end x -----------------------------------")
        while dy**2 > 10 or dy == 0:
            curr_translation[0] += dy*(10**-4)
            move(curr_translation, curr_quaternion, False)
            take_image(False)
            dx, dy = calculate_error()
        print("end y --------------------------------------")

    curr_translation[0] += -0.155
    curr_translation[1] += -0.165
    curr_translation[2] += -.159

    curr_quaternion = [0.066, 0.995, 0.011, 0.075]
    move(curr_translation, curr_quaternion, False)
    rospy.on_shutdown(clean_shutdown)
    #rospy.loginfo("Camera_display node running. Ctrl-c to quit")
    rospy.sleep(1)
