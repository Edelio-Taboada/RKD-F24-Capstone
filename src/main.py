"""
    You will use this script for the final demo.
    Implement your code flow here.
    Your main fucntion can take in different command line arguments to run different parts of your code.
"""
import argparse
import numpy as np
import utils
from motion_planner import TrajectoryFollower
from frankapy import FrankaArm
import time
from calibrate_workspace import WorkspaceCalibrator
from robot_config import RobotConfig
from robot import Robot

'''
# Define default values
parser = argparse.ArgumentParser()
parser.add_argument('--foo', default=1, type=float, help='foo')

# Get the args container with default values
if __name__ == '__main__':
    args = parser.parse_args()  # get arguments from command line
else:
    args = parser.parse_args('')  # get default arguments

# Modify the container, add arguments, change values, etc.
args.foo = 2
args.bar = 3
'''

MAX_ACCELERATIONS = [15, 7.5, 10, 12.5, 15, 20, 20]
SAFE_MAX_ACCELERATIONS = [15, 7.5, 10, 12.5, 15, 20, 20]

two_seconds = np.array([[0, 2]])
five_seconds = np.array([[0, 5]])
ten_seconds = np.array([[0, 10]])

#calibration: post-checkpoint
robot = Robot()

calibrator = WorkspaceCalibrator()

pen_grab_pose = calibrator.calibrate_pen_holders()
ee_at_pen1 = robot.forward_kinematics(pen_grab_pose)
ee_at_pen2 = ee_at_pen1 + something
ee_at_pen3 = ee_at_pen2 + something

whiteboard_pose, whiteboard_T = calibrator.calibrate_whiteboard()


drop_pose = calibrator.calibrate_drop_location()

config = RobotConfig()

HOME = np.array(config.HOME_JOINTS)
ON_MARKER_1 = np.array(pen_grab_pose)
WHITEBOARD_CENTER = np.array(whiteboard_pose)
ABOVE_BIN = np.array(drop_pose)

# HOME_ROTATION = robot.forward_kinematics(HOME)
# HOME_QUATERNION = utils._rotation_to_quaternion(HOME_ROTATION)

# AM1_QUATERNION = pass #TODO: figure out quaternion above marker 1

# #HOME to above marker 1 (AM1)
# SLERP_HOME_2_AM1 = utils._slerp(HOME_QUATERNION, AM1_QUATERNION, two_seconds) 
# #above marker 1 (AM1) to on marker 1 (OM1)
# LERP_AM1_2_OM1 = utils.checkpoint_lerp(np.array([[ABOVE_MARKER_1[i], ON_MARKER_1[i]]for i in range(7)]), two_seconds)
# #on marker 1 (OM1) to above marker 1 (AM1)
# LERP_OM1_2_AM1 = utils.checkpoint_lerp(np.array([[ON_MARKER_1[i], ABOVE_MARKER_1[i]]for i in range(7)]), two_seconds)

# discritized_points_q2_q1 = utils.checkpoint_lerp(q2_q1, two_seconds)

# discritized_points_q1_q3 = utils.checkpoint_lerp(q1_q3, two_seconds)

if __name__ == '__main__':
    fa = FrankaArm()
    tf = TrajectoryFollower()

    fa.reset_joints()
    fa.open_gripper()

    # robot.marker_len(0.1034)

    print("NOT ZEROS!!!!!!!!")
    thetas = fa.get_joints()

    # thetas = np.array([np.pi/2, np.pi/2, np.pi/2, 0, 0, 0, 0])
    print(thetas)

    correct_FK = fa.get_links_transforms(thetas)

    correct_J = fa.get_jacobian(thetas)

    my_J = robot.ef_jacobian(thetas)

    my_FK = robot.forward_kinematics(thetas)
    print("This is the correct FK EF frame from the robot:")
    print(correct_FK)
    print()
    print("and this is ours:")
    print(my_FK)
    print()
    print()
    print("This is the correct Jacobian from the robot:")
    print(correct_J)
    print()
    print("and this is ours:")
    print(my_J)


    # print("ZEROS!!!!!!!!")
    # thetas = np.array([0, 0, 0, 0, 0, 0, 0])
    # robot.change_marker_len(0.2104)

    # # correct_FK = fa.get_links_transforms(thetas)

    # correct_J = fa.get_jacobian(thetas)

    

    # # my_FK = robot.forward_kinematics(thetas)
    # # print("This is the correct FK EF frame from the robot:")
    # # print(correct_FK)
    # # print()
    # # print("and this is ours:")
    # # print(my_FK)
    # # print()
    # # print()
    # print("This is the correct Jacobian from the robot:")
    # print(correct_J)
    # print()
    # print("and this is ours:")

    # my_J = robot.ef_jacobian(thetas)
    # print(my_J)

    # print("diving into the jacobians:")
    # for i in range(7):
    #     print("COLUMN " + str(i))
    #     print("top 3 (linears) from column " + str(i) + ":")
    #     print(str(correct_J[0, i]) + "compared to our -->" + str(my_J[0, i]))
    #     print(str(correct_J[1, i]) + "compared to our -->" + str(my_J[1, i]))
    #     print(str(correct_J[2, i]) + "compared to our -->" + str(my_J[2, i]))
        
    #     print("bottom 3 (angulars) from column " + str(i) + ":")
    #     print(str(correct_J[3, i]) + "compared to our -->" + str(my_J[3, i]))
    #     print(str(correct_J[4, i]) + "compared to our -->" + str(my_J[4, i]))
    #     print(str(correct_J[5, i]) + "compared to our -->" + str(my_J[5, i]))

    # old code:

    # # from home to above marker
    # tf.follow_joint_trajectory(SLERP_HOME_2_AM1.T)
    # time.sleep(2)
    # # from above marker to on marker
    # tf.follow_joint_trajectory(LERP_AM1_2_OM1.T)
    # time.sleep(2)
    # # grab marker
    # fa.close_gripper()
    # # pick up marker
    # tf.follow_joint_trajectory(LERP_OM1_2_AM1.T)
    # time.sleep(2)
    # # bring marker to basket
    # tf.follow_joint_trajectory(discritized_points_q1_q3.T)
    # time.sleep(2)
    # # drop marker
    # fa.open_gripper()

    # return home
    fa.reset_joints()

