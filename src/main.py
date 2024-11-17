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

times = np.array([[5,10]])

ON_MARKER = [0.15444055, 0.29287314, 0.30672833, -2.38176281, -0.40874126, 2.7013571, 1.98458884]
ABOVE_MARKER = [0.16167332, -0.14580426, 0.37693924, -2.44870918, -0.15881228, 2.47492598, 1.78272693]
HOME_POS = [5.35584243e-04, -7.85295146e-01, 2.69296343e-04, -2.35705560e+00, -4.33021669e-04, 1.57136411e+00, 7.85855047e-01]

q0 = np.array(HOME_POS)
q1 = np.array(ABOVE_MARKER)
q2 = np.array(ON_MARKER)

waypointsq0q1 = np.array([[q0[i], q1[i]]for i in range(7)])
waypointsq1q2 = np.array([[q1[i], q2[i]]for i in range(7)])
# print(waypoints)

discritized_points_q0q1 = utils.checkpoint_lerp(waypointsq0q1, times)
discritized_points_q1q2 = utils.checkpoint_lerp(waypointsq1q2, times)
# print(discritized_points[0, :])
# print("break")
# print(discritized_points.T[:, 0])
# print(discritized_points.T[:, 1])

if __name__ == '__main__':
    fa = FrankaArm()
    TrajectoryFollower.follow_joint_trajectory(discritized_points_q0q1)
    TrajectoryFollower.follow_joint_trajectory(discritized_points_q1q2)
    fa.close_gripper()
