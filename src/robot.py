import sys
sys.path.append('../config')
import numpy as np
# from frankapy import FrankaArm
# from robot_config import RobotConfig
# from task_config import TaskConfig

class Robot:
    def __init__(self):
        """Initialize motion planner with robot controller"""
        self.dof = 7
        self.marker_len = 0.107

    def change_marker_len(self, new_len):
        self.marker_len = new_len

    
    def dh_parameter_frames(self, dh_parameters, thetas):
        if thetas.ndim != 1:
            raise ValueError('Expecting a 1D array of joint angles.')

        if thetas.shape[0] != self.dof:
            raise ValueError(f'Invalid number of joints: {thetas.shape[0]} found, expecting {self.dof}')
        
        # --------------- BEGIN STUDENT SECTION ------------------------------------------------
        frames = np.zeros((self.dof + 1, 4, 4))

        for joint in range(1, self.dof + 1):
            theta = thetas[joint-1]
            a = dh_parameters[joint][0]
            alpha = dh_parameters[joint][1]
            d = dh_parameters[joint][2]

            frames[joint-1] = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                                        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                                        [0,             np.sin(alpha),                np.cos(alpha),                             d],
                                        [0,             0,                            0,                                         1]])
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
        
        DH_PARAMS_NO_THETAS = np.array([
            [0,         0,          1/3],
            [0,         -np.pi/2,   0],
            [0,         np.pi/2,    0.316],
            [0.0825,    np.pi/2,    0],
            [-0.0825,   -np.pi/2,   0.384],
            [0,         np.pi/2,    0],
            [0,         np.pi/2,    0],
            [0,         0,          self.marker_len]
        ])
        dh_frames = self.dh_parameter_frames(DH_PARAMS_NO_THETAS, thetas)

        all_frames = np.zeros((self.dof + 1, 4, 4))

        all_frames[0, :, :] = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        for joint in range(1, self.dof + 1):
            all_frames[joint, :, :] = np.matmul(all_frames[joint-1, :, :], dh_frames[joint-1])
        
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

        for i1 in range(0, self.dof):
            # angular part is easier, so we'll start with that
            # given the curent joint we are looking at:
                # we pick out the "previous" fk_frame,
                # and take the first three rows of the 3rd column (the z-column)
            Z_i1 = fk_frames[i1, 0:3, 2]
            jacobian[3:6, i1] = Z_i1

            # now we do linear stuff:
                # first let's grab O_{i-1}
            O_i1 = fk_frames[i1, 0:3, -1]
            O_subtracted = O_6 - O_i1

                # we can use the same Z_i1 we already grabbed for the angle,
                # and now we just take the cross product
            jacobian[0:3, i1] = np.cross(Z_i1, O_subtracted)

        return jacobian



        # raise NotImplementedError("Implement jacobians")
        # --------------- END STUDENT SECTION --------------------------------------------
    

    def pose_error_magnitude(self, curr_pose, target_pose):
        return np.linalg.norm(curr_pose - target_pose)
    
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
        
        if seed_joints.shape != (self.dof):
            raise ValueError(f'Invalid initial_thetas: Expected shape ({self.dof},), got {seed_joints.shape}.')
        
        if seed_joints is None:
            seed_joints = self.robot.arm.get_joints()
        
        # --------------- BEGIN STUDENT SECTION ------------------------------------------------
        # TODO: Implement gradient inverse kinematics
        # thetas is a copy of the original joint configuration
        thetas = seed_joints

        #step size for gradient descent (arbitrary)
        step_size = 0.1

        #once the norm of the computed gradient is less than the stopping condition
        #we stop optimizing
        stopping_condition = 0.00005

        #max number of iterations
        max_iter = 200
        num_iter = 0

        while num_iter < max_iter:
            # [x,y,z,theta] goal
            cost_gradient = np.zeros(self.dof)
            #compute current end effector pose
            ee_pose = self.forward_kinematics(thetas)
            #compute the difference between current position and goal
            distance = self.pose_error_magnitude(ee_pose, target_pose)
            #compute the Jacobian

            #NEED TO IMPLEMENT JACOBIANS
            J = self.ef_jacobian(thetas)
            # compute cost gradient
            cost_gradient = np.transpose(J) @ distance
            
            thetas -= step_size * cost_gradient

            if np.linalg.norm(cost_gradient) < stopping_condition:
                return thetas
            num_iter+=1
        return thetas
        # --------------- END STUDENT SECTION --------------------------------------------------

