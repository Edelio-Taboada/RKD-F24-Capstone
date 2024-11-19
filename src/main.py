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

MAX_ACCELERATIONS = [15, 7.5, 10, 12.5, 15, 20, 20]
SAFE_MAX_ACCELERATIONS = [15, 7.5, 10, 12.5, 15, 20, 20]

two_seconds = np.array([[0, 2]])
five_seconds = np.array([[0, 5]])
ten_seconds = np.array([[0, 10]])

ON_MARKER = [0.1624053, 0.34693658, 0.31160452, -2.28293546, -0.15116802,  2.59795957, 1.43668825]
ABOVE_MARKER = [0.16167332, -0.14580426, 0.37693924, -2.44870918, -0.15881228, 2.47492598, 1.78272693]
HOME_POS = [5.35584243e-04, -7.85295146e-01, 2.69296343e-04, -2.35705560e+00, -4.33021669e-04, 1.57136411e+00, 7.85855047e-01]
ABOVE_BASKET = [-0.09936102, 0.43361226, 0.13748652, -1.88409354, -0.08281602, 1.84765035, 0.82561855]

q0 = np.array(HOME_POS)
q1 = np.array(ABOVE_MARKER)
q2 = np.array(ON_MARKER)

q3 = np.array(ABOVE_BASKET)

q0_q1 = np.array([[q0[i], q1[i]]for i in range(7)])
q1_q2 = np.array([[q1[i], q2[i]]for i in range(7)])
q2_q1 = np.array([[q2[i], q1[i]]for i in range(7)])

q1_q3 = np.array([[q1[i], q3[i]]for i in range(7)])

# print(waypoints)

discritized_points_q0_q1 = utils.checkpoint_lerp(q0_q1, two_seconds)
discritized_points_q1_q2 = utils.checkpoint_lerp(q1_q2, two_seconds)
discritized_points_q2_q1 = utils.checkpoint_lerp(q2_q1, two_seconds)

discritized_points_q1_q3 = utils.checkpoint_lerp(q1_q3, two_seconds)

if __name__ == '__main__':
    fa = FrankaArm()
    tf = TrajectoryFollower()

    fa.reset_joints()
    fa.open_gripper()

    # from home to above marker
    tf.follow_joint_trajectory(discritized_points_q0_q1.T)
    time.sleep(2)
    # from above marker to on marker
    tf.follow_joint_trajectory(discritized_points_q1_q2.T)
    time.sleep(2)
    # grab marker
    fa.close_gripper()
    # pick up marker
    tf.follow_joint_trajectory(discritized_points_q2_q1.T)
    time.sleep(2)
    # bring marker to basket
    tf.follow_joint_trajectory(discritized_points_q1_q3.T)
    time.sleep(2)
    # drop marker
    fa.open_gripper()

    # return home
    fa.reset_joints()

