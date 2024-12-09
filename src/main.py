"""
    You will use this script for the final demo.
    Implement your code flow here.
    Your main fucntion can take in different command line arguments to run different parts of your code.
"""
import argparse
from autolab_core import RigidTransform
from motion_planner import TrajectoryFollower, TrajectoryGenerator
from calibrate_workspace import WorkspaceCalibrator
import sys
sys.path.append("../config")

import numpy as np
from robot import Robot
from robot_config import RobotConfig
from task_config import TaskConfig
from frankapy import FrankaArm

import time

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

# Call the program with passed arguments
fa = FrankaArm()
kinematics = Robot()
calibrator = WorkspaceCalibrator()

home_pose_q = RobotConfig.HOME_JOINTS
home_xyz = kinematics.forward_kinematics(home_pose_q)[:3,3,-1]
pen_R = kinematics.forward_kinematics(home_pose_q)[:3,:3,-1]

fa.reset_joints()
fa.open_gripper()

pen_seperation = 0.058
vertical_seperation = 0.15

calibrator.calibrate_pen_holders()
pen_1_xyz = np.load('pen_holder_pose.npy',allow_pickle=True)
pen_2_xyz = pen_1_xyz + np.array([pen_seperation,0,0])
pen_3_xyz = pen_1_xyz - np.array([pen_seperation,0,0])
pen_4_xyz = pen_1_xyz + np.array([pen_seperation/2.0,0,0])
pen_5_xyz = pen_1_xyz - np.array([pen_seperation/2.0,0,0])
pen_6_xyz = pen_1_xyz + np.array([0,0.035,0])

pen_1_above_xyz = pen_1_xyz + np.array([0,0,vertical_seperation])
pen_2_above_xyz = pen_2_xyz + np.array([0,0,vertical_seperation])
pen_3_above_xyz = pen_3_xyz + np.array([0,0,vertical_seperation])
pen_4_above_xyz = pen_4_xyz + np.array([0,0,vertical_seperation])
pen_5_above_xyz = pen_5_xyz + np.array([0,0,vertical_seperation])
pen_6_above_xyz = pen_6_xyz + np.array([0,0,vertical_seperation])

pre_pick_ready_q = np.array([0.3874, -0.0308, 0.2433, -2.0961, 0.027, 2.0726, 1.3682])
pre_pick_ready_pose = kinematics.forward_kinematics(pre_pick_ready_q)[:,:,-1]
pre_pick_ready_xyz = pre_pick_ready_pose[:3,3]

fa.reset_joints()
input(f"Press enter to grab pen...")
fa.close_gripper()

calibrator.calibrate_whiteboard()
whiteboard_pose = np.load('whiteboard_pose.npy',allow_pickle=True)

whiteboard_xyz = whiteboard_pose[:3,3]
whiteboard_R = whiteboard_pose[:3,:3]

point_above_whiteboard_xyz = whiteboard_pose[:3,-1] - (0.05*whiteboard_pose[:3,2])
whiteboard_POI_1 = whiteboard_pose[:3,-1] + (0.12*whiteboard_pose[:3,0]) # Creates a new point 10cm to the left of the center of the board
whiteboard_POI_2 = whiteboard_pose[:3,-1] - (0.12*whiteboard_pose[:3,0]) # Creates a new point 10cm to the right of the center of the board
whiteboard_POI_3 = whiteboard_pose[:3,-1] + (0.05*whiteboard_pose[:3,1]) # Creates a new point 5cm up from the center of the board
whiteboard_POI_4 = whiteboard_pose[:3,-1] - (0.05*whiteboard_pose[:3,1]) # Creates a new point 5cm down from the center of the board

au_1 = whiteboard_xyz + (0.05*whiteboard_pose[:3,0]) + (0.05*whiteboard_pose[:3,1])
au_2 = whiteboard_xyz + (0.05*whiteboard_pose[:3,0]) + (-0.05*whiteboard_pose[:3,1])
au_3 = whiteboard_xyz + (0.015*whiteboard_pose[:3,0]) + (0.06*whiteboard_pose[:3,1])
au_4 = whiteboard_xyz + (0.03*whiteboard_pose[:3,0]) + (0.04*whiteboard_pose[:3,1])
au_5 = whiteboard_xyz + (0.015*whiteboard_pose[:3,0]) + (0.025*whiteboard_pose[:3,1])
au_6 = whiteboard_xyz + (0.015*whiteboard_pose[:3,0]) + (-0.01*whiteboard_pose[:3,1])
au_7 = whiteboard_xyz + (0.015*whiteboard_pose[:3,0]) + (-0.05*whiteboard_pose[:3,1])
au_8 = whiteboard_xyz + (-0.015*whiteboard_pose[:3,0]) + (-0.01*whiteboard_pose[:3,1])
au_9 = whiteboard_xyz + (-0.015*whiteboard_pose[:3,0]) + (-0.05*whiteboard_pose[:3,1])
au_10 = whiteboard_xyz + (0*whiteboard_pose[:3,0]) + (0.1*whiteboard_pose[:3,1])
au_11 = whiteboard_xyz + (-0.05*whiteboard_pose[:3,0]) + (0.05*whiteboard_pose[:3,1])
au_12 = whiteboard_xyz + (-0.025*whiteboard_pose[:3,0]) + (0.06*whiteboard_pose[:3,1])
au_13 = whiteboard_xyz + (-0.025*whiteboard_pose[:3,0]) + (0.025*whiteboard_pose[:3,1])
au_14 = whiteboard_xyz + (0.0325*whiteboard_pose[:3,0]) + (-0.07*whiteboard_pose[:3,1])
au_15 = whiteboard_xyz + (-0.0325*whiteboard_pose[:3,0]) + (-0.07*whiteboard_pose[:3,1])
au_16 = whiteboard_xyz + (-0.05*whiteboard_pose[:3,0]) + (-0.05*whiteboard_pose[:3,1])
au_17 = whiteboard_xyz + (-0.04*whiteboard_pose[:3,0]) + (0.04*whiteboard_pose[:3,1])
au_18 = whiteboard_xyz + (0.06*whiteboard_pose[:3,0]) + (0.06*whiteboard_pose[:3,1])
au_19 = whiteboard_xyz + (0.09*whiteboard_pose[:3,0]) + (0.05*whiteboard_pose[:3,1])
au_20 = whiteboard_xyz + (0.09*whiteboard_pose[:3,0]) + (-0.025*whiteboard_pose[:3,1])
au_21 = whiteboard_xyz + (0.06*whiteboard_pose[:3,0]) + (-0.04*whiteboard_pose[:3,1])
au_22 = whiteboard_xyz + (0.05*whiteboard_pose[:3,0]) + (-0.025*whiteboard_pose[:3,1])

calibrator.calibrate_drop_location()
drop_xyz = np.load('drop_bin_pose.npy',allow_pickle=True)
drop_R = np.array([
    [0.4945,-0.5327,-0.6868],
    [-0.7354,-0.6776,-0.0039],
    [-0.4633,0.5070,-0.7269]
])

tg = TrajectoryGenerator()
tf = TrajectoryFollower()
max_vel = RobotConfig.MAX_VELOCITY
max_acc = RobotConfig.MAX_ACCELERATION

fa.reset_joints()
fa.open_gripper()

# Pick up pen 1
current_joint = np.array(home_pose_q)
home_to_pre_pick_1 = tg.generate_straight_line(home_xyz, pen_1_above_xyz, current_joint, pen_R, duration=3)
tf.follow_joint_trajectory(home_to_pre_pick_1)
time.sleep(0.3)
current_joint = home_to_pre_pick_1[-1]

pre_pick_1_to_pick_1 = tg.generate_straight_line(pen_1_above_xyz, pen_1_xyz, current_joint, pen_R, duration=1.5)
tf.follow_joint_trajectory(pre_pick_1_to_pick_1)
time.sleep(0.3)
fa.close_gripper()
time.sleep(0.3)
current_joint = pre_pick_1_to_pick_1[-1]

pick_1_to_pre_pick_1 = tg.generate_straight_line(pen_1_xyz, pen_1_above_xyz, current_joint, pen_R, duration=1.5)
tf.follow_joint_trajectory(pick_1_to_pre_pick_1)
time.sleep(0.3)
current_joint = pick_1_to_pre_pick_1[-1]

pre_pick_1_to_home = tg.generate_trapezoidal_trajectory(current_joint, home_pose_q, max_vel, max_acc, duration=3)
tf.follow_joint_trajectory(pre_pick_1_to_home)
time.sleep(0.3)
current_joint = pre_pick_1_to_home[-1]

draw_R = whiteboard_R

# Draw line with pen 1
home_to_above_au1 = tg.generate_straight_line(home_xyz, au_1-(0.05*whiteboard_pose[:3,2]), current_joint, pen_R, draw_R, duration=4)
tf.follow_joint_trajectory(home_to_above_au1)
time.sleep(0.3)
current_joint = home_to_above_au1[-1]

above_au1_to_au1 = tg.generate_straight_line(au_1-(0.05*whiteboard_pose[:3,2]), au_1, current_joint, draw_R, duration=1)
tf.follow_joint_trajectory(above_au1_to_au1)
time.sleep(0.3)
current_joint = above_au1_to_au1[-1]

au1_to_au2 = tg.generate_straight_line(au_1, au_2, current_joint, draw_R, duration=2)
tf.follow_joint_trajectory(au1_to_au2)
time.sleep(0.3)
current_joint = au1_to_au2[-1]

au2_to_above_au3 = tg.generate_straight_line(au_2, au_3-(0.05*whiteboard_pose[:3,2]), current_joint, draw_R, duration=2)
tf.follow_joint_trajectory(au2_to_above_au3)
time.sleep(0.3)
current_joint = au2_to_above_au3[-1]

above_au3_to_au3 = tg.generate_straight_line(au_3-(0.05*whiteboard_pose[:3,2]), au_3, current_joint, draw_R, duration=1)
tf.follow_joint_trajectory(above_au3_to_au3)
time.sleep(0.3)
current_joint = above_au3_to_au3[-1]

points = np.vstack((au_3, au_4, au_5))
au3_to_au4_to_au5 = tg.generate_curve_3(points, current_joint, draw_R, duration=4)
tf.follow_joint_trajectory(au3_to_au4_to_au5)
time.sleep(0.3)
current_joint = au3_to_au4_to_au5[-1]

au5_to_above_au6 = tg.generate_straight_line(au_5, au_6-(0.05*whiteboard_pose[:3,2]), current_joint, draw_R, duration=2)
tf.follow_joint_trajectory(au5_to_above_au6)
time.sleep(0.3)
current_joint = au5_to_above_au6[-1]

above_au6_to_au6 = tg.generate_straight_line(au_6-(0.05*whiteboard_pose[:3,2]), au_6, current_joint, draw_R, duration=1)
tf.follow_joint_trajectory(above_au6_to_au6)
time.sleep(0.3)
current_joint = above_au6_to_au6[-1]

au6_to_au7 = tg.generate_straight_line(au_6, au_7, current_joint, draw_R, duration=1)
tf.follow_joint_trajectory(au6_to_au7)
time.sleep(0.3)
current_joint = au6_to_au7[-1]

au7_to_above_au8 = tg.generate_straight_line(au_7, au_8-(0.05*whiteboard_pose[:3,2]), current_joint, draw_R, duration=2)
tf.follow_joint_trajectory(au7_to_above_au8)
time.sleep(0.3)
current_joint = au7_to_above_au8[-1]

above_au8_to_au8 = tg.generate_straight_line(au_8-(0.05*whiteboard_pose[:3,2]), au_8, current_joint, draw_R, duration=1)
tf.follow_joint_trajectory(above_au8_to_au8)
time.sleep(0.3)
current_joint = above_au8_to_au8[-1]

au8_to_au9 = tg.generate_straight_line(au_8, au_9, current_joint, draw_R, duration=1)
tf.follow_joint_trajectory(au8_to_au9)
time.sleep(0.3)
current_joint = au8_to_au9[-1]

au9_to_above_board = tg.generate_straight_line(au_9, point_above_whiteboard_xyz, current_joint, draw_R, duration=1)
tf.follow_joint_trajectory(au9_to_above_board)
time.sleep(0.3)
current_joint = au9_to_above_board[-1]

# Drop pen 1
above_board_to_home = tg.generate_trapezoidal_trajectory(current_joint, home_pose_q, max_vel, max_acc, 3)
tf.follow_joint_trajectory(above_board_to_home)
time.sleep(0.3)
current_joint = above_board_to_home[-1]

decision = input(f"Type yes to put back to pen holder, type anything else to put into drop bin: ")
if decision == "yes":
    current_joint = np.array(home_pose_q)
    home_to_pre_pick_4 = tg.generate_straight_line(home_xyz, pen_4_above_xyz, current_joint, pen_R, duration=3)
    tf.follow_joint_trajectory(home_to_pre_pick_4)
    time.sleep(0.3)
    current_joint = home_to_pre_pick_4[-1]

    pre_pick_4_to_pick_4 = tg.generate_straight_line(pen_4_above_xyz, pen_4_xyz, current_joint, pen_R, duration=2)
    tf.follow_joint_trajectory(pre_pick_4_to_pick_4)
    time.sleep(0.3)
    fa.open_gripper()
    time.sleep(0.3)
    current_joint = pre_pick_4_to_pick_4[-1]

    pick_4_to_pre_pick_4 = tg.generate_straight_line(pen_4_xyz, pen_4_above_xyz, current_joint, pen_R, duration=2)
    tf.follow_joint_trajectory(pick_4_to_pre_pick_4)
    time.sleep(0.3)
    current_joint = pick_4_to_pre_pick_4[-1]

    pre_pick_4_to_home = tg.generate_trapezoidal_trajectory(current_joint, home_pose_q, max_vel, max_acc, duration=3)
    tf.follow_joint_trajectory(pre_pick_4_to_home)
    time.sleep(0.3)
    current_joint = pre_pick_4_to_home[-1]

else:
    home_to_drop = tg.generate_straight_line(home_xyz, drop_xyz, current_joint, pen_R, drop_R, duration=4)
    tf.follow_joint_trajectory(home_to_drop)
    time.sleep(0.3)
    current_joint = home_to_drop[-1]
    fa.open_gripper()

    drop_to_home = tg.generate_trapezoidal_trajectory(current_joint, home_pose_q, max_vel, max_acc, 4)
    tf.follow_joint_trajectory(drop_to_home)
    time.sleep(0.3)

# Pick up pen 2
current_joint = np.array(home_pose_q)
home_to_pre_pick_2 = tg.generate_straight_line(home_xyz, pen_2_above_xyz, current_joint, pen_R, duration=3)
tf.follow_joint_trajectory(home_to_pre_pick_2)
time.sleep(0.3)
current_joint = home_to_pre_pick_2[-1]

pre_pick_2_to_pick_2 = tg.generate_straight_line(pen_2_above_xyz, pen_2_xyz, current_joint, pen_R, duration=1.5)
tf.follow_joint_trajectory(pre_pick_2_to_pick_2)
time.sleep(0.3)
fa.close_gripper()
time.sleep(0.3)
current_joint = pre_pick_2_to_pick_2[-1]

pick_2_to_pre_pick_2 = tg.generate_straight_line(pen_2_xyz, pen_2_above_xyz, current_joint, pen_R, duration=1.5)
tf.follow_joint_trajectory(pick_2_to_pre_pick_2)
time.sleep(0.3)
current_joint = pick_2_to_pre_pick_2[-1]

pre_pick_2_to_home = tg.generate_trapezoidal_trajectory(current_joint, home_pose_q, max_vel, max_acc, duration=3)
tf.follow_joint_trajectory(pre_pick_2_to_home)
time.sleep(0.3)
current_joint = pre_pick_2_to_home[-1]

# Draw with pen 2
home_to_above_au1 = tg.generate_straight_line(home_xyz, au_1-(0.05*whiteboard_pose[:3,2]), current_joint, pen_R, draw_R, duration=4)
tf.follow_joint_trajectory(home_to_above_au1)
time.sleep(0.3)
current_joint = home_to_above_au1[-1]

above_au1_to_au1 = tg.generate_straight_line(au_1-(0.05*whiteboard_pose[:3,2]), au_1, current_joint, draw_R, duration=1)
tf.follow_joint_trajectory(above_au1_to_au1)
time.sleep(0.3)
current_joint = above_au1_to_au1[-1]

points = np.vstack((au_1, au_10, au_11))
au1_to_au10_to_au11 = tg.generate_curve_3(points, current_joint, draw_R, duration=4)
tf.follow_joint_trajectory(au1_to_au10_to_au11)
time.sleep(0.3)
current_joint = au1_to_au10_to_au11[-1]

au11_to_above_au3 = tg.generate_straight_line(au_11, au_3-(0.05*whiteboard_pose[:3,2]), current_joint, draw_R, duration=1)
tf.follow_joint_trajectory(au11_to_above_au3)
time.sleep(0.3)
current_joint = au11_to_above_au3[-1]

above_au3_to_au3 = tg.generate_straight_line(au_3-(0.05*whiteboard_pose[:3,2]), au_3, current_joint, draw_R, duration=1)
tf.follow_joint_trajectory(above_au3_to_au3)
time.sleep(0.3)
current_joint = above_au3_to_au3[-1]

au3_to_au12 = tg.generate_straight_line(au_3, au_12, current_joint, draw_R, duration=2)
tf.follow_joint_trajectory(au3_to_au12)
time.sleep(0.3)
current_joint = au3_to_au12[-1]

au12_to_above_au5 = tg.generate_straight_line(au_12, au_5-(0.05*whiteboard_pose[:3,2]), current_joint, draw_R, duration=1)
tf.follow_joint_trajectory(au12_to_above_au5)
time.sleep(0.3)
current_joint = au12_to_above_au5[-1]

above_au5_to_au5 = tg.generate_straight_line(au_5-(0.05*whiteboard_pose[:3,2]), au_5, current_joint, draw_R, duration=1)
tf.follow_joint_trajectory(above_au5_to_au5)
time.sleep(0.3)
current_joint = above_au5_to_au5[-1]

au5_to_au13 = tg.generate_straight_line(au_5, au_13, current_joint, draw_R, duration=2)
tf.follow_joint_trajectory(au5_to_au13)
time.sleep(0.3)
current_joint = au5_to_au13[-1]

au13_to_above_au6 = tg.generate_straight_line(au_13, au_6-(0.05*whiteboard_pose[:3,2]), current_joint, draw_R, duration=1)
tf.follow_joint_trajectory(au13_to_above_au6)
time.sleep(0.3)
current_joint = au13_to_above_au6[-1]

above_au6_to_au6 = tg.generate_straight_line(au_6-(0.05*whiteboard_pose[:3,2]), au_6, current_joint, draw_R, duration=1)
tf.follow_joint_trajectory(above_au6_to_au6)
time.sleep(0.3)
current_joint = above_au6_to_au6[-1]

au6_to_au8 = tg.generate_straight_line(au_6, au_8, current_joint, draw_R, duration=2)
tf.follow_joint_trajectory(au6_to_au8)
time.sleep(0.3)
current_joint = au6_to_au8[-1]

au8_to_above_au2 = tg.generate_straight_line(au_8, au_2-(0.05*whiteboard_pose[:3,2]), current_joint, draw_R, duration=1)
tf.follow_joint_trajectory(au8_to_above_au2)
time.sleep(0.3)
current_joint = au8_to_above_au2[-1]

above_au2_to_au2 = tg.generate_straight_line(au_2-(0.05*whiteboard_pose[:3,2]), au_2, current_joint, draw_R, duration=1)
tf.follow_joint_trajectory(above_au2_to_au2)
time.sleep(0.3)
current_joint = above_au2_to_au2[-1]

points = np.vstack((au_2, au_14, au_7))
au2_to_au14_to_au7 = tg.generate_curve_3(points, current_joint, draw_R, duration=3)
tf.follow_joint_trajectory(au2_to_au14_to_au7)
time.sleep(0.3)
current_joint = au2_to_au14_to_au7[-1]

au7_to_above_au9 = tg.generate_straight_line(au_7, au_9-(0.05*whiteboard_pose[:3,2]), current_joint, draw_R, duration=1)
tf.follow_joint_trajectory(au7_to_above_au9)
time.sleep(0.3)
current_joint = au7_to_above_au9[-1]

above_au9_to_au9 = tg.generate_straight_line(au_9-(0.05*whiteboard_pose[:3,2]), au_9, current_joint, draw_R, duration=1)
tf.follow_joint_trajectory(above_au9_to_au9)
time.sleep(0.3)
current_joint = above_au9_to_au9[-1]

points = np.vstack((au_9, au_15, au_16))
au9_to_au15_to_au16 = tg.generate_curve_3(points, current_joint, draw_R, duration=3)
tf.follow_joint_trajectory(au9_to_au15_to_au16)
time.sleep(0.3)
current_joint = au9_to_au15_to_au16[-1]

au16_to_above_center = tg.generate_straight_line(au_16, point_above_whiteboard_xyz, current_joint, draw_R, duration=1)
tf.follow_joint_trajectory(au16_to_above_center)
time.sleep(0.3)
current_joint = au16_to_above_center[-1]

# Drop pen 2
above_board_to_home = tg.generate_trapezoidal_trajectory(current_joint, home_pose_q, max_vel, max_acc, 3)
tf.follow_joint_trajectory(above_board_to_home)
time.sleep(0.3)
current_joint = above_board_to_home[-1]

decision = input(f"Type yes to put back to pen holder, type anything else to put into drop bin: ")
if decision == "yes":
    current_joint = np.array(home_pose_q)
    home_to_pre_pick_5 = tg.generate_straight_line(home_xyz, pen_5_above_xyz, current_joint, pen_R, duration=3)
    tf.follow_joint_trajectory(home_to_pre_pick_5)
    time.sleep(0.3)
    current_joint = home_to_pre_pick_5[-1]

    pre_pick_5_to_pick_5 = tg.generate_straight_line(pen_5_above_xyz, pen_5_xyz, current_joint, pen_R, duration=2)
    tf.follow_joint_trajectory(pre_pick_5_to_pick_5)
    time.sleep(0.3)
    fa.open_gripper()
    time.sleep(0.3)
    current_joint = pre_pick_5_to_pick_5[-1]

    pick_5_to_pre_pick_5 = tg.generate_straight_line(pen_5_xyz, pen_5_above_xyz, current_joint, pen_R, duration=2)
    tf.follow_joint_trajectory(pick_5_to_pre_pick_5)
    time.sleep(0.3)
    current_joint = pick_5_to_pre_pick_5[-1]

    pre_pick_5_to_home = tg.generate_trapezoidal_trajectory(current_joint, home_pose_q, max_vel, max_acc, duration=3)
    tf.follow_joint_trajectory(pre_pick_5_to_home)
    time.sleep(0.3)
    current_joint = pre_pick_5_to_home[-1]

else:
    home_to_drop = tg.generate_straight_line(home_xyz, drop_xyz, current_joint, pen_R, drop_R, duration=4)
    tf.follow_joint_trajectory(home_to_drop)
    time.sleep(0.3)
    current_joint = home_to_drop[-1]
    fa.open_gripper()

    drop_to_home = tg.generate_trapezoidal_trajectory(current_joint, home_pose_q, max_vel, max_acc, 4)
    tf.follow_joint_trajectory(drop_to_home)
    time.sleep(0.3)


# Pick up pen 3
home_to_pre_pick_3 = tg.generate_straight_line(home_xyz, pen_3_above_xyz, current_joint, pen_R, duration=3)
tf.follow_joint_trajectory(home_to_pre_pick_3)
time.sleep(0.3)
current_joint = home_to_pre_pick_3[-1]

pre_pick_3_to_pick_3 = tg.generate_straight_line(pen_3_above_xyz, pen_3_xyz, current_joint, pen_R, duration=1.5)
tf.follow_joint_trajectory(pre_pick_3_to_pick_3)
time.sleep(0.3)
fa.close_gripper()
time.sleep(0.3)
current_joint = pre_pick_3_to_pick_3[-1]

pick_3_to_pre_pick_3 = tg.generate_straight_line(pen_3_xyz, pen_3_above_xyz, current_joint, pen_R, duration=1.5)
tf.follow_joint_trajectory(pick_3_to_pre_pick_3)
time.sleep(0.3)
current_joint = pick_3_to_pre_pick_3[-1]

pre_pick_3_to_home = tg.generate_trapezoidal_trajectory(current_joint, home_pose_q, max_vel, max_acc, duration=3)
tf.follow_joint_trajectory(pre_pick_3_to_home)
time.sleep(0.3)
current_joint = pre_pick_3_to_home[-1]

# Draw with pen 3
home_to_above_au11 = tg.generate_straight_line(home_xyz, au_11-(0.05*whiteboard_pose[:3,2]), current_joint, pen_R, draw_R, duration=4)
tf.follow_joint_trajectory(home_to_above_au11)
time.sleep(0.3)
current_joint = home_to_above_au11[-1]

above_au11_to_au11 = tg.generate_straight_line(au_11-(0.05*whiteboard_pose[:3,2]), au_11, current_joint, draw_R, duration=1)
tf.follow_joint_trajectory(above_au11_to_au11)
time.sleep(0.3)
current_joint = above_au11_to_au11[-1]

au11_to_au16 = tg.generate_straight_line(au_11, au_16, current_joint, draw_R, duration=2)
tf.follow_joint_trajectory(au11_to_au16)
time.sleep(0.3)
current_joint = au11_to_au16[-1]

au16_to_above_au12 = tg.generate_straight_line(au_16, au_12-(0.05*whiteboard_pose[:3,2]), current_joint, pen_R, draw_R, duration=1)
tf.follow_joint_trajectory(au16_to_above_au12)
time.sleep(0.3)
current_joint = au16_to_above_au12[-1]

above_au12_to_au12 = tg.generate_straight_line(au_12-(0.05*whiteboard_pose[:3,2]), au_12, current_joint, draw_R, duration=1)
tf.follow_joint_trajectory(above_au12_to_au12)
time.sleep(0.3)
current_joint = above_au12_to_au12[-1]

points = np.vstack((au_12, au_17, au_13))
au12_to_au17_to_au13 = tg.generate_curve_3(points, current_joint, draw_R, duration=3)
tf.follow_joint_trajectory(au12_to_au17_to_au13)
time.sleep(0.3)
current_joint = au12_to_au17_to_au13[-1]

au13_to_above_au1 = tg.generate_straight_line(au_13, au_1-(0.05*whiteboard_pose[:3,2]), current_joint, pen_R, draw_R, duration=1)
tf.follow_joint_trajectory(au13_to_above_au1)
time.sleep(0.3)
current_joint = au13_to_above_au1[-1]

above_au1_to_au1 = tg.generate_straight_line(au_1-(0.05*whiteboard_pose[:3,2]), au_1, current_joint, draw_R, duration=1)
tf.follow_joint_trajectory(above_au1_to_au1)
time.sleep(0.3)
current_joint = above_au1_to_au1[-1]

points = np.vstack((au_1, au_18, au_19, au_20, au_21, au_22))
au1_to_au22 = tg.generate_curve_3(points, current_joint, draw_R, duration=8)
tf.follow_joint_trajectory(au1_to_au22)
time.sleep(0.3)
current_joint = au1_to_au22[-1]

au22_to_above_center = tg.generate_straight_line(au_22, point_above_whiteboard_xyz, current_joint, pen_R, draw_R, duration=1)
tf.follow_joint_trajectory(au22_to_above_center)
time.sleep(0.3)
current_joint = au22_to_above_center[-1]


# Drop pen 3
above_board_to_home = tg.generate_trapezoidal_trajectory(current_joint, home_pose_q, max_vel, max_acc, 3)
tf.follow_joint_trajectory(above_board_to_home)
time.sleep(0.3)
current_joint = above_board_to_home[-1]

decision = input(f"Type yes to put back to pen holder, type anything else to put into drop bin: ")
if decision == "yes":
    home_to_pre_pick_6 = tg.generate_straight_line(home_xyz, pen_6_above_xyz, current_joint, pen_R, duration=3)
    tf.follow_joint_trajectory(home_to_pre_pick_6)
    time.sleep(0.3)
    current_joint = home_to_pre_pick_6[-1]

    pre_pick_6_to_pick_6 = tg.generate_straight_line(pen_6_above_xyz, pen_6_xyz, current_joint, pen_R, duration=2)
    tf.follow_joint_trajectory(pre_pick_6_to_pick_6)
    time.sleep(0.3)
    fa.open_gripper()
    time.sleep(0.3)
    current_joint = pre_pick_6_to_pick_6[-1]

    pick_6_to_pre_pick_6 = tg.generate_straight_line(pen_6_xyz, pen_6_above_xyz, current_joint, pen_R, duration=2)
    tf.follow_joint_trajectory(pick_6_to_pre_pick_6)
    time.sleep(0.3)
    current_joint = pick_6_to_pre_pick_6[-1]

    pre_pick_6_to_home = tg.generate_trapezoidal_trajectory(current_joint, home_pose_q, max_vel, max_acc, duration=3)
    tf.follow_joint_trajectory(pre_pick_6_to_home)
    time.sleep(0.3)
    current_joint = pre_pick_6_to_home[-1]

else:
    home_to_drop = tg.generate_straight_line(home_xyz, drop_xyz, current_joint, pen_R, drop_R, duration=5)
    tf.follow_joint_trajectory(home_to_drop)
    time.sleep(0.3)
    current_joint = home_to_drop[-1]
    fa.open_gripper()

    drop_to_home = tg.generate_trapezoidal_trajectory(current_joint, home_pose_q, max_vel, max_acc, 5)
    tf.follow_joint_trajectory(drop_to_home)
    time.sleep(0.3)