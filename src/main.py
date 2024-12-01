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

# Call the program with passed arguments

MAX_ACCELERATIONS = [15, 7.5, 10, 12.5, 15, 20, 20]
SAFE_MAX_ACCELERATIONS = [15, 7.5, 10, 12.5, 15, 20, 20]

two_seconds = np.array([[0, 2]])
five_seconds = np.array([[0, 5]])
ten_seconds = np.array([[0, 10]])

#calibration: post-checkpoint
robot = Robot()

calibrator = WorkspaceCalibrator()
pen_positions = calibrator.calibrate_pen_holders()
whiteboard_pose = calibrator.calibrate_whiteboard()
drop_pose = calibrator.calibrate_drop_location()

config = RobotConfig()

HOME = np.array(config.HOME_JOINTS)
ON_MARKER_1 = np.array(pen_positions)
WHITEBOARD_CENTER = np.array(whiteboard_pose)
ABOVE_BIN = np.array(drop_pose)

HOME_ROTATION = robot.forward_kinematics(HOME)
HOME_QUATERNION = utils._rotation_to_quaternion(HOME_ROTATION)

AM1_QUATERNION = pass #TODO: figure out quaternion above marker 1

#HOME to above marker 1 (AM1)
SLERP_HOME_2_AM1 = utils._slerp(HOME_QUATERNION, AM1_QUATERNION, two_seconds) 
#above marker 1 (AM1) to on marker 1 (OM1)
LERP_AM1_2_OM1 = utils.checkpoint_lerp(np.array([[ABOVE_MARKER_1[i], ON_MARKER_1[i]]for i in range(7)]), two_seconds)
#on marker 1 (OM1) to above marker 1 (AM1)
LERP_OM1_2_AM1 = utils.checkpoint_lerp(np.array([[ON_MARKER_1[i], ABOVE_MARKER_1[i]]for i in range(7)]), two_seconds)

discritized_points_q2_q1 = utils.checkpoint_lerp(q2_q1, two_seconds)

discritized_points_q1_q3 = utils.checkpoint_lerp(q1_q3, two_seconds)

if __name__ == '__main__':
    fa = FrankaArm()
    tf = TrajectoryFollower()

    fa.reset_joints()
    fa.open_gripper()

    # from home to above marker
    tf.follow_joint_trajectory(SLERP_HOME_2_AM1.T)
    time.sleep(2)
    # from above marker to on marker
    tf.follow_joint_trajectory(LERP_AM1_2_OM1.T)
    time.sleep(2)
    # grab marker
    fa.close_gripper()
    # pick up marker
    tf.follow_joint_trajectory(LERP_OM1_2_AM1.T)
    time.sleep(2)
    # bring marker to basket
    tf.follow_joint_trajectory(discritized_points_q1_q3.T)
    time.sleep(2)
    # drop marker
    fa.open_gripper()

    # return home
    fa.reset_joints()

