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
    part1_YM = math.sqrt(np.linalg.det(part1_square))

    part2_J = rob.ef_jacobian(np.array(part2_angles))
    part2_square = np.matmul(part2_J, part2_J.T)
    part2_YM = math.sqrt(np.linalg.det(part2_square))

    print("Yoshikawa manipulability term μ for part 1 is")
    print(part1_YM)
    print("Yoshikawa manipulability term μ for part 2 is")
    print(part2_YM)

