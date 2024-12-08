import sys
sys.path.append('../config')
import numpy as np
import utils
from scipy.spatial.transform import Rotation as R
# from frankapy import FrankaArm
# from robot_config import RobotConfig
# from task_config import TaskConfig

class Robot:
    # def __init__(self, L_frame, R_frame, M_frame):
    def __init__(self):
        """Initialize motion planner with robot controller"""
        # frames are from calibrate_workspace
        self.dof = 7
        self.marker_len = 0.107

        self.DH_PARAMS_NO_THETAS = np.array([
            [0,         0,          1/3],
            [0,         -np.pi/2,   0],
            [0,         np.pi/2,    0.316],
            [0.0825,    np.pi/2,    0],
            [-0.0825,   -np.pi/2,   0.384],
            [0,         np.pi/2,    0],
            [0.088,     np.pi/2,    0],
            [0,         0,          0.1034],
            [0,         0,          0.107]
        ])

        # given initialized frames, create the normal vector for the whiteboard
        # L_points = L_frame[0:3, -1]
        # R_points = R_frame[0:3, -1]
        # M_points = M_frame[0:3, -1]

        # A = L_points - M_points
        # B = R_points - M_points
        # self.wb_normal = np.cross(A, B)

    def change_marker_len(self, new_len):
        self.marker_len = new_len

    def dh_frame_from_vals(self, alpha, a, d, theta):
        
        c_a = np.cos(alpha)
        s_a = np.sin(alpha)
        c_t = np.cos(theta)
        s_t = np.sin(theta)

        return np.array([
            [c_t,      -s_t,        0,      a       ],
            [c_a*s_t,   c_a*c_t,   -s_a,   -d*s_a  ],
            [s_a*s_t,   c_t*s_a,    c_a,    d*c_a   ],
            [0,         0,          0,      1       ]
        ])

    
    def dh_parameter_frames(self, dh_parameters, thetas):
        if thetas.ndim != 1:
            raise ValueError('Expecting a 1D array of joint angles.')

        if thetas.shape[0] != self.dof:
            raise ValueError(f'Invalid number of joints: {thetas.shape[0]} found, expecting {self.dof}')
        
        # --------------- BEGIN STUDENT SECTION ------------------------------------------------
        frames = np.zeros((self.dof + 2, 4, 4))

        for joint in range(0, self.dof):
            theta = thetas[joint]
            a = dh_parameters[joint][0]
            alpha = dh_parameters[joint][1]
            d = dh_parameters[joint][2]

            frames[joint] = self.dh_frame_from_vals(alpha, a, d, theta)
        
        theta = -np.pi/4
        a = 0
        alpha = 0
        d = 0.107 + 0.1034

        frames[self.dof] = self.dh_frame_from_vals(alpha, a, d, theta)

        theta = 0
        a = 0
        alpha = 0
        d = 0

        frames[self.dof+1] = self.dh_frame_from_vals(alpha, a, d, theta)
        # end effector frame
        # theta = 0
        # a = 0
        # alpha = 0
        # d = self.marker_len

        # frames[self.dof] = self.dh_frame_from_vals(alpha, a, d, theta)

        return frames
    
    def forward_kinematics(self, thetas):
        """
        Compute foward kinematics
        
        Your implementation should:
        1. Compute transformation matrices for each frame using DH parameters
        2. Compute end-effector pose
        
        Parameters
        ----------
        dh_parameters: np.ndarray
            DH parameters (you can choose to apply the offset to the tool flange, center of gripper, or the pen tip)
        thetas : np.ndarray
            All joint angles
            
        Returns
        -------
        np.ndarray
            End-effector pose
        """
        if thetas.ndim != 1:
            raise ValueError('Expecting a 1D array of joint angles.')

        if thetas.shape[0] != self.dof:
            raise ValueError(f'Invalid number of joints: {thetas.shape[0]} found, expecting {self.dof}')
        
        # --------------- BEGIN STUDENT SECTION ------------------------------------------------
        
        dh_frames = self.dh_parameter_frames(self.DH_PARAMS_NO_THETAS, thetas)
        # print("MY DH FRAMES")
        # print(dh_frames)

        all_frames = np.zeros((self.dof + 1, 4, 4))

        all_frames[0, :, :] = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        for joint in range(1, self.dof + 1):
            all_frames[joint, :, :] = np.matmul(all_frames[joint-1, :, :], dh_frames[joint-1])
            if (joint == self.dof) :
                # print("bing bong")
                # print(dh_frames[joint])
                # print(dh_frames[joint+1])
                all_frames[joint, :, :] = np.matmul(all_frames[joint, :, :], dh_frames[joint])
                all_frames[joint, :, :] = np.matmul(all_frames[joint, :, :], dh_frames[joint+1])
        
        return all_frames
        # --------------- END STUDENT SECTION --------------------------------------------------

    
    def ef_jacobian(self, thetas):
        """
        Compute the Jacobian for the end effector frame.
        NOTE:   this is a 6 by self.dof matrix (6x7)
                The columns correspond to each joint,
                The rows correspond to each cartesian movement type
                                        (x, y, z, roll, pitch, yaw)

        Parameters
        ----------
        thetas : np.ndarray
            All joint angles
            
        Returns
        -------
        np.ndarray
            jacobian
        """
        if thetas.shape != (self.dof,):
            raise ValueError(f'Invalid thetas: Expected shape ({self.dof},), got {thetas.shape}.')
        
        # epsilon = 0.001

        # --------------- BEGIN STUDENT SECTION ----------------------------------------
        # TODO: Implement the numerical computation of the Jacobians

        # Hints:
        # - Perturb each joint angle by epsilon and compute the forward kinematics.
        # - Compute numerical derivatives for x, y, and positions.
        # - Determine the rotational component based on joint contributions.

        # using the formula from lecture 19
        # I don't think this is the same as numerical IK

        jacobian = np.zeros((6, self.dof))

        fk_frames = self.forward_kinematics(thetas)

        # last frame, first three rows, last column
        O_6 = fk_frames[-1, 0:3, -1]
        # print(O_6)

        # print(O_6)

        for i1 in range(0, self.dof):
            # angular part is easier, so we'll start with that
            # given the curent joint we are looking at:
                # we pick out the "previous" fk_frame,
                # and take the first three rows of the 3rd column (the z-column)
            Z_i1 = fk_frames[i1+1, 0:3, 2]
            jacobian[3:6, i1] = Z_i1

            # now we do linear stuff:
                # first let's grab O_{i-1}
            O_i1 = fk_frames[i1+1, 0:3, -1]
            O_subtracted = O_6 - O_i1

                # we can use the same Z_i1 we already grabbed for the angle,
                # and now we just take the cross product
            jacobian[0:3, i1] = np.cross(Z_i1, O_subtracted)

        return jacobian



        # raise NotImplementedError("Implement jacobians")
        # --------------- END STUDENT SECTION --------------------------------------------
    

    def error_from_poses(self, cur_pose, target_pose):
        # print("POSE:")
        # print(frame)
        
        # print("rotation")
        # ee_rotation = frame[0:3, 0:3]
        # print(ee_rotation)
        R_cur = cur_pose[0:3, 0:3]
        R_target = target_pose[0:3, 0:3]

        R_needed = np.matmul(R_cur.T, R_target)

        rotation = R.from_matrix(R_needed)
        # axis angle representation
        
        axis_angle = rotation.as_rotvec()
        angle = np.linalg.norm(axis_angle)  # Rotation angle in radians
        axis = axis_angle / angle if angle != 0 else [0, 0, 0]  # Normalized axis of rotation
        error_angle = np.matmul(R_cur, axis)

        # ee_RPY = axis
        # print("RPY")
        # print(ee_RPY)
        # print("XYZ:")
        subtracted = cur_pose - target_pose
        ee_XYZ = subtracted[0:3, 3]
        # print(ee_XYZ)

        ee_converted = np.zeros((6, 1))
        ee_converted[0:3, 0] = ee_XYZ
        ee_converted[3:6, 0] = error_angle

        # print(ee_converted)
        # print()
        
        return ee_converted
    
    def _inverse_kinematics(self, target_pose, seed_joints):
        """
        Compute inverse kinematics using Jacobian pseudo-inverse method.
        
        Your implementation should:
        1. Start from seed joints
        2. Iterate until convergence or max iterations
        3. Check joint limits and singularities
        4. Return None if no valid solution
        
        Parameters
        ----------
        target_pose : 4x4 np.ndarray
            Desired end-effector pose
        seed_joints : np.ndarray
            Initial joint configuration
            
        Returns
        -------
        np.ndarray or None
            Joint angles that achieve target pose, or None if not found
            
        Hints
        -----
        - Use get_pose() from robot arm
        - Implement a helper function to track pose error magnitude for convergence
        - The iteration parameters are defined in RobotConfig and TaskConfig, feel free to update them
        """
        
        if seed_joints.shape != (self.dof,):
            raise ValueError(f'Invalid initial_thetas: Expected shape ({self.dof},), got {seed_joints.shape}.')
        
        if seed_joints is None:
            seed_joints = self.robot.arm.get_joints()
        
        # --------------- BEGIN STUDENT SECTION ------------------------------------------------
        # TODO: Implement gradient inverse kinematics
        # thetas is a copy of the original joint configuration
        
        print("Trying to get to target_pose:")
        print(target_pose)
        print()
        print()

        thetas = seed_joints

        #step size for gradient descent (arbitrary)
        step_size = 0.01

        #once the norm of the computed gradient is less than the stopping condition
        #we stop optimizing
        stopping_condition = 0.00005

        #max number of iterations
        max_iter = 1000
        num_iter = 0

        while num_iter < max_iter:
            # [x,y,z,theta] goal
            cost_gradient = np.zeros(self.dof)
            #compute current end effector pose
            # ee_pose = self.frame_to_pose(self.forward_kinematics(thetas)[-1])

            #compute the difference between current position and goal
            # error = self.pose_error_magnitude(ee_pose, target_pose)
            error = self.error_from_poses(self.forward_kinematics(thetas)[-1], target_pose)
            #compute the Jacobian

            #NEED TO IMPLEMENT JACOBIANS
            J = self.ef_jacobian(thetas)
            # print(J)
            # print(J.shape)
            # compute cost gradient
            # print(error)
            # print(error.shape)
            cost_gradient = np.matmul(np.transpose(J), error)

            # print(":D")
            print(cost_gradient.T[0])
            # print(cost_gradient)
            # print(cost_gradient.T[0].shape)
            print(thetas)
            # print(thetas.shape)
            
            thetas -= (step_size * cost_gradient.T[0])

            # print("********************************************************************")
            # print("********************************************************************")
            print(np.linalg.norm(cost_gradient))
            # print(np.linalg.norm(cost_gradient.T[0]))
            # print("********************************************************************")
            # print("********************************************************************")

            if np.linalg.norm(cost_gradient) < stopping_condition:
                # print("FINISHED YAY")
                # print(num_iter)
                return thetas
            num_iter+=1
        
        print("RAN OUT OF ITERATIONS")
        print(np.linalg.norm(cost_gradient))
        print("target pose: ")
        print(target_pose)
        return thetas
        # --------------- END STUDENT SECTION --------------------------------------------------
