from robot import Robot
import numpy as np
import math

part1_angles = [np.pi/2, np.pi/2, np.pi/2, 0, 0, 0, 0]
part2_angles = [0, 0, 0, 0, 0, 0, 0]

if __name__ == '__main__':
    rob = Robot()

    ef_size = 0
    rob.change_marker_len(ef_size)

    part1_J = rob.ef_jacobian(np.array(part1_angles))
    part1_square = np.matmul(part1_J, part1_J.T)
    part1_det = math.fabs(np.linalg.det(part1_square))
    part1_YM = math.sqrt(part1_det)

    part2_J = rob.ef_jacobian(np.array(part2_angles))
    part2_square = np.matmul(part2_J, part2_J.T)
    part2_det = math.fabs(np.linalg.det(part2_square))
    part2_YM = math.sqrt(part2_det)

    print("Yoshikawa manipulability term μ for part 1 (if end effector is 0) is")
    print(part1_YM)
    print("Yoshikawa manipulability term μ for part 2 (if end effector is 0) is")
    print(part2_YM)
    print()

    ef_size = 0.1034
    rob.change_marker_len(ef_size)

    part1_J = rob.ef_jacobian(np.array(part1_angles))
    part1_square = np.matmul(part1_J, part1_J.T)
    part1_det = math.fabs(np.linalg.det(part1_square))
    part1_YM = math.sqrt(part1_det)

    part2_J = rob.ef_jacobian(np.array(part2_angles))
    part2_square = np.matmul(part2_J, part2_J.T)
    part2_det = math.fabs(np.linalg.det(part2_square))
    part2_YM = math.sqrt(part2_det)

    print("Yoshikawa manipulability term μ for part 1 (if end effector is 0.1034) is")
    print(part1_YM)
    print("Yoshikawa manipulability term μ for part 2 (if end effector is 0.1034) is")
    print(part2_YM)
    print()

    ef_size = 0.107
    rob.change_marker_len(ef_size)

    part1_J = rob.ef_jacobian(np.array(part1_angles))
    part1_square = np.matmul(part1_J, part1_J.T)
    part1_det = math.fabs(np.linalg.det(part1_square))
    part1_YM = math.sqrt(part1_det)

    part2_J = rob.ef_jacobian(np.array(part2_angles))
    part2_square = np.matmul(part2_J, part2_J.T)
    part2_det = math.fabs(np.linalg.det(part2_square))
    part2_YM = math.sqrt(part2_det)

    print("Yoshikawa manipulability term μ for part 1 (if end effector is 0.107) is")
    print(part1_YM)
    print("Yoshikawa manipulability term μ for part 2 (if end effector is 0.107) is")
    print(part2_YM)

