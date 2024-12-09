import sys
sys.path.append("../config")

import numpy as np
from scipy.interpolate import CubicSpline
import utils

from autolab_core import RigidTransform
from robot_config import RobotConfig
from frankapy import FrankaArm
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from franka_interface_msgs.msg import SensorDataGroup
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import JointPositionSensorMessage, ShouldTerminateSensorMessage
import rospy
from robot import Robot

class TrajectoryGenerator:
    def __init__(self, dt=0.02):
        self.dt = dt
        self.max_vel = RobotConfig.MAX_VELOCITY
        self.max_acc = RobotConfig.MAX_ACCELERATION

    def generate_cartesian_waypoints(self, start_pose, end_pose, duration):
        """
        Generate a Cartesian trajectory as a series of waypoints using linear interpolation 
        for position and Spherical Linear Interpolation (SLERP) for orientation.
        
        This method calculates waypoints at regular intervals over the specified duration
        to create a smooth motion from the starting pose to the ending pose. Each waypoint
        includes both position and orientation data, ensuring that the trajectory is not only
        spatially accurate but also maintains correct alignment throughout the movement.

        Parameters
        ----------
        start_pose : RigidTransform
            The starting pose of the trajectory, including position and orientation.
        end_pose : RigidTransform
            The ending pose of the trajectory, including position and orientation.
        duration : float
            The total time over which the trajectory should be executed.

        Returns
        -------
        list of RigidTransform
            A list of RigidTransform objects representing the waypoints of the trajectory.
            Each waypoint is spaced at an interval of 20ms, as defined by `self.dt`.

        Notes
        -----
        - Waypoints are calculated at 20ms intervals.
        - Linear interpolation is used for the translational component of the trajectory.
        - SLERP is utilized for the rotational component to ensure smooth transitions between orientations.
        - The trajectory generation assumes constant velocity, which may not be suitable for all applications
        where acceleration and deceleration phases are required.

        Hints
        -----
        - The number of points, `n_points`, is determined by the duration divided by the timestep `self.dt`.
        - Use `np.linspace` to generate times from 0 to `duration` for the waypoints.
        - For position, interpolate linearly between `start_pose.translation` and `end_pose.translation`.
        - For rotation, convert rotation matrices to quaternions and use SLERP from `q0` to `q1`.
        - Ensure each waypoint is constructed with the interpolated position and orientation and respects
        the frame specifications of the `start_pose`.
        """
        n_points = int(duration / self.dt) + 1
        times = np.linspace(0, duration, n_points)
        t_values = np.linspace(0, 1, n_points)

        try:
            start_pos = start_pose.translation
            end_pos = end_pose.translation
        except AttributeError:
            start_pos = start_pose.position
            end_pos = end_pose.position
            
        start_pos = np.array(start_pos)
        end_pos = np.array(end_pos)

        positions = np.outer(1 - t_values, start_pos) + np.outer(t_values, end_pos)

        q0 = utils._rotation_to_quaternion(start_pose.rotation)
        q1 = utils._rotation_to_quaternion(end_pose.rotation)

        q0 = q0 / np.linalg.norm(q0)
        q1 = q1 / np.linalg.norm(q1)

        quaternions = []
        for t in t_values:
            q_interp = utils._slerp(q0, q1, t)
            q_interp /= np.linalg.norm(q_interp)  
            quaternions.append(q_interp)

        waypoints = []
        for i in range(n_points):
            position = positions[i]
            rotation_matrix = utils._quaternion_to_rotation(quaternions[i])
            waypoint = RigidTransform(
                rotation=rotation_matrix,
                translation=position,
                from_frame=start_pose.from_frame,
                to_frame=start_pose.to_frame
            )
            waypoints.append(waypoint)

        return waypoints
    
    def generate_straight_line(self, start_point, end_point, current_joint, start_R, end_R=None, duration=None):
        """
        This function creates a smooth straight-line trajectory for the robot's end-effector to follow.

        Parameters
        ----------
        start_point : array_like
            The starting point of the line in Cartesian space.
        end_point : array_like
            The ending point of the line in Cartesian space.
        duration : float, optional
            The total duration over which the line should be drawn. If not specified,
            the speed should default to a pre-defined value.
            
        Return
        ------
        array_like
            Input to either generate_cartesian_waypoints or follow_cartesian_trajectory

        Raises
        ------
        NotImplementedError
            This function needs to be implemented.

        Notes
        -----
        - The method needs to handle both the translational and rotational dynamics to ensure
        that the end-effector is properly aligned during the motion.
        - If duration is not provided, calculate it based on a default speed and the distance between points.

        Hints
        -----
        - Use linear interpolation to compute intermediate points along the line.
        - Optionally, incorporate orientation interpolation if the end-effector's orientation is critical.
        - This method should eventually call a function to actually move the robot's joints based on the
        interpolated points.
        """
        kinematics = Robot()
        times = np.arange(0, duration + self.dt, self.dt)
        t_ends = np.array([times[0],times[-1]])
        x0,y0,z0 = start_point
        x1,y1,z1 = end_point
        bc_type = ((1,0),(1,0))

        cs_x = CubicSpline(t_ends,[x0,x1],bc_type=bc_type)
        cs_y = CubicSpline(t_ends,[y0,y1],bc_type=bc_type)
        cs_z = CubicSpline(t_ends,[z0,z1],bc_type=bc_type)

        # trajectory_xyz = np.linspace(start_point,end_point,len(times))
        smooth_x = cs_x(times)
        smooth_y = cs_y(times)
        smooth_z = cs_z(times)
        trajectory_xyz = np.column_stack((smooth_x,smooth_y,smooth_z))
        trajectory = np.zeros((len(times)-1,7))

        if end_R is None:
            R = start_R
        
            current_transform = kinematics._get_transform(R,start_point)
            target_transform = current_transform

            for i in range(len(times)-1):
                target_transform[:3,-1] = trajectory_xyz[i+1]                           
                current_joint = kinematics._inverse_kinematics(target_transform,current_joint)
                trajectory[i] = current_joint

        else:
            start_quaternion = utils._rotation_to_quaternion(start_R)
            end_quaternion = utils._rotation_to_quaternion(end_R)

            trajectory_R = np.zeros((3,3,len(times)+1))
            for i in range(len(times)):
                current_t = times[i]
                current_quaternion = utils._slerp(start_quaternion,end_quaternion,current_t/duration)
                current_R = utils._quaternion_to_rotation(current_quaternion)
                trajectory_R[:,:,i+1] = current_R

            current_transform = kinematics._get_transform(start_R,start_point)
            target_transform = current_transform

            for i in range(len(times)-1):
                target_transform[:3,:3] = trajectory_R[:,:,i+1]
                target_transform[:3,-1] = trajectory_xyz[i+1]                           
                current_joint = kinematics._inverse_kinematics(target_transform,current_joint)
                trajectory[i] = current_joint
                
        print(trajectory)
        return trajectory

        
        '''
        print(f"Input Start Point: {start_point}")
        print(f"Input End Point: {end_point}")
        print(f"Given Trajectory_xyz start: {trajectory_xyz[0]}")
        print(f"Given Trajectory_xyz end: {trajectory_xyz[-1]}")
        print("\n")
        print("Trajectories:")
        for i in range(trajectory.shape[0]):
            print(kinematics.forward_kinematics(trajectory[i])[:3,3,-1])
        print("")
        print(f"Output Start Pose:")
        print(kinematics.forward_kinematics(trajectory[0])[:3,3,-1])
        print(f"Output End Pose:")
        print(kinematics.forward_kinematics(trajectory[-1])[:3,3,-1])
        '''

    def generate_curve_3(self, points, current_joint, R, duration=5):
        kinematics = Robot()
        times = np.arange(0,duration + self.dt, self.dt)
        trajectory = np.zeros((len(times),7))
        trajectory[0] = current_joint

        t = np.linspace(0, duration + self.dt, len(points))
        x = points[:,0]
        y = points[:,1]
        z = points[:,2]
        
        bc_type = ((1,0),(1,0))

        cs_x = CubicSpline(t,x,bc_type=bc_type)
        cs_y = CubicSpline(t,y,bc_type=bc_type)
        cs_z = CubicSpline(t,z,bc_type=bc_type)

        x_curve = cs_x(times)
        y_curve = cs_y(times)
        z_curve = cs_z(times)

        trajectory_xyz = np.column_stack((x_curve,y_curve,z_curve))

        current_transform = kinematics._get_transform(R,points[0])
        target_transform = current_transform

        for i in range(len(times)-1):
            target_transform[:3,-1] = trajectory_xyz[i+1]                           
            current_joint = kinematics._inverse_kinematics(target_transform,current_joint)
            trajectory[i+1] = current_joint

        return trajectory


    
    def generate_trapezoidal_trajectory(self, q_start, q_end, max_vel, max_acc, duration):
        """
        Generate trajectory with trapezoidal velocity profile.
        
        From writeup:
        - Must have proper acceleratio
    MAX_DRAWING_RETRIES = 2n/deceleration
        - Must maintain smooth motion
        - Must output at 20ms intervals
        
        Parameters
        ----------
        q_start : np.ndarray
            Starting joint configuration
        q_end : np.ndarray
            Ending joint configuration
        max_vel : float
            Maximum allowed velocity
        max_acc : float
            Maximum allowed acceleration
        duration : float
            Total trajectory duration
            
        Returns
        -------
        np.ndarray
            Array of shape (n_points, 7) with joint angles at 20ms intervals
            
        Hints
        -----
        - Use three phases: acceleration, constant velocity, deceleration
        - Calculate proper acceleration time
        - Ensure smooth transitions between phases
        """
      
        delta_q = np.subtract(q_end,q_start)
        n_joints = len(q_start)
        total_distance = np.linalg.norm(delta_q)
        t_acc = duration / 2
        a = 4 * delta_q / duration ** 2
        v_max = a * t_acc

        t_samples = np.arange(0, duration + self.dt, self.dt)
        trajectory = np.zeros((len(t_samples), n_joints))

        for i in range(n_joints):
            a_i = a[i]
            q0 = q_start[i]
            v_max_i = v_max[i]
            for idx, t in enumerate(t_samples):
                if t <= t_acc:
                    q = q0 + 0.5 * a_i * t ** 2
                else:
                    t_dec = t - t_acc
                    q_half = q0 + 0.5 * a_i * t_acc ** 2
                    q = q_half + v_max_i * t_dec - 0.5 * a_i * t_dec ** 2
                trajectory[idx, i] = q

        return trajectory

    def interpolate_trajectory(self, times, trajectory):
        """
        Interpolate a given trajectory to match the required 20ms control rate.
        This function will create a new trajectory that aligns with the timing requirements
        for dynamic control of the robot.

        Parameters
        ----------
        times : array_like
            An array of time points for which the trajectory needs interpolation.
        trajectory : array_like
            An array of waypoints or joint configurations that define the initial trajectory.

        Raises
        ------
        NotImplementedError
            This function needs to be implemented.

        Notes
        -----
        - The output trajectory should have waypoints spaced at exactly 20ms intervals.
        - This method is critical for ensuring smooth motion and adhering to the control loop timing.

        Hints
        -----
        - Consider using linear interpolation for simple cases or cubic spline interpolation for smoother trajectories.
        - Ensure the interpolated trajectory respects the physical and operational limits of the robot, such as
        maximum velocities and accelerations.
        """

        times = np.array(times)
        trajectory = np.array(trajectory)
        n_joints = trajectory.shape[1]

        t_interpolated = np.arange(times[0], times[-1] + self.dt, self.dt)
        interpolated_trajectory = np.zeros((len(t_interpolated), n_joints))

        for joint_idx in range(n_joints):
            joint_positions = trajectory[:, joint_idx]
            n = len(times)
            h = np.diff(times)
            b = np.diff(joint_positions) / h

            A = np.zeros((n, n))
            rhs = np.zeros(n)

            A[0, 0] = 1
            A[-1, -1] = 1
            rhs[0] = 0
            rhs[-1] = 0

            for i in range(1, n - 1):
                A[i, i - 1] = h[i - 1]
                A[i, i] = 2 * (h[i - 1] + h[i])
                A[i, i + 1] = h[i]
                rhs[i] = 3 * (b[i] - b[i - 1])

            c = np.linalg.solve(A, rhs)
            a_coeffs = np.zeros(n - 1)
            b_coeffs = np.zeros(n - 1)
            d_coeffs = joint_positions[:-1]

            for i in range(n - 1):
                a_coeffs[i] = (c[i + 1] - c[i]) / (3 * h[i])
                b_coeffs[i] = b[i] - h[i] * (2 * c[i] + c[i + 1]) / 3


            interpolated_positions = np.zeros_like(t_interpolated)
            for idx, t in enumerate(t_interpolated):
                # Find the right interval
                if t <= times[0]:
                    i = 0
                elif t >= times[-1]:
                    i = n - 2
                else:
                    i = np.searchsorted(times, t) - 1

                dt = t - times[i]
                interpolated_positions[idx] = (a_coeffs[i] * dt ** 3 +
                                            b_coeffs[i] * dt ** 2 +
                                            c[i] * dt +
                                            d_coeffs[i])

            interpolated_trajectory[:, joint_idx] = interpolated_positions
        
        return interpolated_trajectory

class TrajectoryFollower:
    def __init__(self):
        self.dt = 0.02  # Required 20ms control loop
        self.fa = FrankaArm()
        
    def follow_joint_trajectory(self, joint_trajectory):
        """
        Follow a joint trajectory using dynamic control.
        
        From writeup: Must have 20ms between waypoints and maintain smooth motion
        
        Parameters
        ----------
        joint_trajectory : np.ndarray
            Array of shape (N, 7) containing joint angles for each timestep
        """
        rospy.loginfo('Initializing Sensor Publisher')
        pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)
        rate = rospy.Rate(1 / self.dt)

        rospy.loginfo('Publishing joints trajectory...')
        # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
        self.fa.goto_joints(joint_trajectory[0], duration=1000, dynamic=True, buffer_time=10)
        init_time = rospy.Time.now().to_time()
        for i in range(1, joint_trajectory.shape[0]):
            traj_gen_proto_msg = JointPositionSensorMessage(
                id=i, timestamp=rospy.Time.now().to_time() - init_time, 
                joints=joint_trajectory[i]
            )
            ros_msg = make_sensor_group_msg(
                trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                    traj_gen_proto_msg, SensorDataMessageType.JOINT_POSITION)
            )
            
            rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
            pub.publish(ros_msg)
            rate.sleep()

        # Stop the skill
        # Alternatively can call fa.stop_skill()
        term_proto_msg = ShouldTerminateSensorMessage(timestamp=rospy.Time.now().to_time() - init_time, should_terminate=True)
        ros_msg = make_sensor_group_msg(
            termination_handler_sensor_msg=sensor_proto2ros_msg(
                term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
            )
        pub.publish(ros_msg)

        rospy.loginfo('Done')