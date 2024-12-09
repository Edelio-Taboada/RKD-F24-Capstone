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
import paths
import test_paths

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

'''----------------constants---------------------'''
MAX_ACCELERATIONS = [15, 7.5, 10, 12.5, 15, 20, 20]
SAFE_MAX_ACCELERATIONS = [15, 7.5, 10, 12.5, 15, 20, 20]
PEN_OFFSET = 0.03175       # distances measured in meters

two_seconds = np.array([[0, 2]])
five_seconds = np.array([[0, 5]])
ten_seconds = np.array([[0, 10]])

'''----------------calibration--------------------'''
robot = Robot()
fa = FrankaArm()

calibrator = WorkspaceCalibrator()

pen_grab_pose = calibrator.calibrate_pen_holders()
on_marker_1_ee = np.eye(4)
on_marker_1_ee[:3,:3] = pen_grab_pose.rotation
on_marker_1_ee[:3,3] = pen_grab_pose.translation
ee_at_pen2 = on_marker_1_ee + np.array([[0,0,0,PEN_OFFSET],
                                    [0,0,0,0],
                                    [0,0,0,0],
                                    [0,0,0,0]])
ee_at_pen3 = ee_at_pen2 + np.array([[0,0,0,PEN_OFFSET],
                                    [0,0,0,0],
                                    [0,0,0,0],
                                    [0,0,0,0]])

whiteboard_pose = calibrator.calibrate_whiteboard()
whiteboard_quat = whiteboard_pose.quaternion

drop_pose = calibrator.calibrate_drop_location()

config = RobotConfig()

home_joints = np.array(config.HOME_JOINTS)
home_quat = utils._rotation_to_quaternion(robot.forward_kinematics(home_joints)[:3,:3])

above_marker_1_ee = robot.forward_kinematics(on_marker_1_ee) + np.array([[0,0,0,0],
                                                                         [0,0,0,0],
                                                                         [0,0,0,config.PEN_LENGTH],
                                                                         [0,0,0,0]])
above_marker_1_joints = robot._inverse_kinematics(above_marker_1_ee, home_joints)
above_marker_1_quat = utils._rotation_to_quaternion(above_marker_1_ee[:3,:3])

on_marker_1_joints = robot._inverse_kinematics(on_marker_1_ee,home_joints)

whiteboard_center_ee = np.eye(4)
whiteboard_center_ee[:3,:3] = whiteboard_pose.rotation
whiteboard_center_ee[:3,3] = whiteboard_pose.translation
whiteboard_center_joints = robot._inverse_kinematics(whiteboard_center_ee,home_joints)
whiteboard_center_quat = utils._rotation_to_quaternion(whiteboard_center_ee[:3,:3])

above_bin_ee = np.eye(4)
above_bin_ee[:3,:3] = drop_pose.rotation
above_bin_ee[:3,3] = drop_pose.translation
above_bin_joints = robot._inverse_kinematics(above_bin_ee,home_joints)
above_bin_quat = utils._rotation_to_quaternion(above_bin_ee[:3,:3])


'''----------------define paths between poses------------------'''
###############  DRAW WITH MARKER 1  ##############################
# home --> above marker 1 --> on marker 1 --> close gripper
home_to_am1 = utils._slerp(home_quat, above_marker_1_quat, five_seconds)
am1_to_om1 = utils.checkpoint_lerp(np.array([[above_marker_1_joints[i], on_marker_1_joints[i]]for i in range(7)]), five_seconds)

# on marker 1 --> above marker 1 --> home
om1_to_am1 = utils.checkpoint_lerp(np.array([[on_marker_1_joints[i], above_marker_1_joints[i]]for i in range(7)]), five_seconds)
am1_to_home = utils._slerp(above_marker_1_quat, home_quat, five_seconds)

# home --> whiteboard center --> draw path --> home
home_to_wbc = utils._slerp(home_quat, whiteboard_center_quat, five_seconds)


# otf_path = "./fonts/Milanello.otf"
# glyph_data = paths.extract_glyph_paths(otf_path)
    
# letter = "B"
# vector_path = glyph_data.get(letter)
# discretized_points = test_paths.discretize_vector_path(vector_path, resolution=100)
# ee_points = utils.whiteboard_2d_to_3d(robot, discretized_points, whiteboard_center_joints, whiteboard_T)

# drawing_trajectory = np.zeros((7, len(list(discretized_points))))

# for i in range(len(list(ee_points))):
#     ee_point = ee_points[i]
#     drawing_joints = robot._inverse_kinematics(ee_point)
#     drawing_trajectory[i,:] = drawing_joints

# wb_end_pose = drawing_trajectory[-1,:]
wb_R = whiteboard_pose[:3,:3]
wb_T = whiteboard_pose[:3,3]
wb_translation = np.array([0.05, 0.05, 0, 0])


wb_translated_ee = np.eye(4)
wb_translated_ee[:4,3] = whiteboard_pose @ wb_translation
wb_translated_ee[:3,:3] = wb_R

wb_end_joints = robot._inverse_kinematics(wb_translated_ee,whiteboard_center_joints)

wb_draw = utils.checkpoint_lerp(np.array([[whiteboard_center_joints[i], wb_end_joints[i]]for i in range(7)]), five_seconds)

wb_to_home = utils.checkpoint_lerp(np.array([[wb_end_joints[i], home_joints[i]]for i in range(7)]), five_seconds)

# home --> above bin --> open gripper --> home
home_to_ab = utils._slerp(home_quat, above_bin_quat, five_seconds)
ab_to_home = utils.checkpoint_lerp(above_bin_quat, home_quat, five_seconds)


###############  DRAW WITH MARKER 2  ##############################
# home --> above marker 2 --> on marker 2 --> close gripper 

# on marker 2 --> above marker 2 --> home

# home --> whiteboard center --> draw path --> home

# home --> above bin --> open gripper --> home


###############  DRAW WITH MARKER 3  ##############################
# home --> above marker 3 --> on marker 3 --> close gripper 

# on marker 3 --> above marker 3 --> home

# home --> whiteboard center --> draw path --> home

# home --> above bin --> open gripper --> home


















# HOME_ROTATION = robot.forward_kinematics(HOME)[:3,:3]
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

    # home --> above marker 1
    tf.follow_joint_trajectory(home_to_am1.T)
    time.sleep(5)
    # above marker 1 --> on marker 1
    tf.follow_joint_trajectory(am1_to_om1.T)
    time.sleep(5)
    # grab marker
    fa.close_gripper()
    # on marker 1 --> above marker 1
    tf.follow_joint_trajectory(om1_to_am1.T)
    time.sleep(5)
    # above marker 1 --> home
    tf.follow_joint_trajectory(am1_to_home.T)
    time.sleep(5)
    # home --> whiteboard center
    tf.follow_joint_trajectory(home_to_wbc.T)
    time.sleep(5)
    # draw on the whiteboard
    tf.follow_joint_trajectory(wb_draw.T)
    time.sleep(10)
    # whiteboard --> home
    tf.follow_joint_trajectory(wb_to_home.T)
    time.sleep(5)
    # home --> above bin
    tf.follow_joint_trajectory(home_to_ab.T)
    time.sleep(5)
    # drop marker in bin
    fa.open_gripper()
    # above bin --> home
    tf.follow_joint_trajectory(ab_to_home.T)
    time.sleep(5)



    # return home
    fa.reset_joints()

