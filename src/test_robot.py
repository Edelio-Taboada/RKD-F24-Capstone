from robot import Robot
import numpy as np 
import utils

ON_MARKER = [0.1624053, 0.34693658, 0.31160452, -2.28293546, -0.15116802,  2.59795957, 1.43668825]
ABOVE_MARKER = [0.16167332, -0.14580426, 0.37693924, -2.44870918, -0.15881228, 2.47492598, 1.78272693]
HOME_POS = [5.35584243e-04, -7.85295146e-01, 2.69296343e-04, -2.35705560e+00, -4.33021669e-04, 1.57136411e+00, 7.85855047e-01]
ABOVE_BASKET = [-0.09936102, 0.43361226, 0.13748652, -1.88409354, -0.08281602, 1.84765035, 0.82561855]

MY_TEST = [0.40708775, 0.47630726, -0.0367942, -2.18650105, 0.13904405, 2.74513102, -0.76728278]

# MARKER_LEN = 0.107 #m
MARKER_LEN = 0



if __name__ == '__main__':
    rob = Robot()

    # Testing FK:
    thetas = np.array(ON_MARKER)
    print("ON the marker, we have:")
    pose_on_marker = rob.forward_kinematics(thetas)[-1]
    print(pose_on_marker)
    print()

    thetas = np.array(ABOVE_MARKER)
    print("ABOVE the marker, we have:")
    pose_above_marker = rob.forward_kinematics(thetas)[-1]
    print(pose_above_marker)
    print()

    thetas = np.array(HOME_POS)
    print("at the HOME POS, we have:")
    pose_at_home = rob.forward_kinematics(thetas)[-1]
    print(pose_at_home)
    print()


    # printing jacobian
    rob.change_marker_len(0)
    zeros = np.array([0, 0, 0, 0, 0, 0, 0])
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!DH:")
    dh_frames = rob.dh_parameter_frames(rob.DH_PARAMS_NO_THETAS, zeros)
    print(dh_frames)
    print("*************FK:")
    print(rob.forward_kinematics(zeros))
    print("-------------J:")
    print(rob.ef_jacobian(zeros))
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    print("Going from above the marker at ")
    print(ABOVE_MARKER)

    rob.change_marker_len(0.107)
    IK_test1 = rob._inverse_kinematics(pose_on_marker, np.array(ABOVE_MARKER))
    print("to on the marker at ")
    print(IK_test1)
    print(rob.forward_kinematics(IK_test1)[-1])
    print("which should be close to ")
    print(ON_MARKER)
    print(pose_on_marker)

