import numpy as np
import math
from scipy.spatial.transform import Rotation as R



def trajectory_trap_vel(waypoints, times, frequency=(1/0.02), duty_cycle=0.25):
    """
    Returns a matrix of joint angles, where each column represents a single
    timestamp. These joint angles form trapezoidal velocity trajectory segments,
    hitting waypoints[:, i] at times[i].

    Args:
    waypoints (np.array): Matrix of waypoints; each column represents a single
                          waypoint in joint space, and each row represents a particular joint.
    times (np.array): Row vector that indicates the time each of the waypoints should
                      be reached. The number of columns should equal the number of waypoints,
                      and should be monotonically increasing.
    frequency (float): The control frequency at which this trajectory should be played,
                       and therefore the number of columns per second of playback.
    duty_cycle (float): The duty cycle for the trapezoidal velocity profile.

    Returns:
    np.array: Matrix of joint angles forming the trajectory.
    """
    # Number of joints
    num_joints = waypoints.shape[0]
    # Number of waypoints
    num_waypoints = waypoints.shape[1]
    # Number of segments between waypoints
    num_segments = num_waypoints - 1

    if times.shape != (1, num_waypoints):
        print(times.shape, num_waypoints)
        raise ValueError('Size of times vector is incorrect!')

    if num_waypoints < 2:
        raise ValueError('Insufficient number of waypoints.')

    if not isinstance(frequency, (int, float)) or frequency < 5:
        raise ValueError('Invalid control frequency (must be at least 5Hz)')

    if duty_cycle < 0 or duty_cycle > 0.5:
        raise ValueError('Invalid duty cycle!')

    # Calculate number of points per segment
    num_points_per_segment = []
    for segment in range(num_segments):
        dt = times[0, segment + 1] - times[0, segment]
        num_points_per_segment.append(int(dt * frequency))

    # Pre-allocate trajectory matrix
    trajectory = np.zeros((num_joints, int(np.sum(num_points_per_segment))))

    # Fill in trajectory segment-by-segment
    segment_start_point = 0
    for segment in range(num_segments):
        points_in_segment = num_points_per_segment[segment]
        segment_end_point = segment_start_point + points_in_segment

        num_ramp_points = int(duty_cycle * points_in_segment)
        ramp_time = (times[0, segment + 1] - times[0, segment]) * duty_cycle

        # --------------- BEGIN STUDENT SECTION ----------------------------------
        # TODO: Calculate the maximum velocity for this segment
        vm = (waypoints[:,segment+1]-waypoints[:,segment])/(times[:,segment+1]-times[:,segment]-ramp_time)
        
        # TODO: Fill in the points for this segment of the trajectory
        # You need to implement the trapezoidal velocity profile here
        # Hint: Use three phases: ramp up, constant velocity, and ramp down

        # Example structure (you need to fill in the correct calculations):
        for joint in range(num_joints):
            q0 = waypoints[joint, segment]
            qf = waypoints[joint, segment + 1]
            v_max = vm[joint]
            t_b = (points_in_segment-num_ramp_points-1)/frequency
            t_a = num_ramp_points/frequency
            t0 = t_a-ramp_time
            tf=t_b+ramp_time

            for i in range(points_in_segment):
                t = i / frequency
                if i <= num_ramp_points:
                    q = q0 + 0.5*v_max/ramp_time*(t-t0)**2
                    # q=0
                elif i > points_in_segment - num_ramp_points:
                    q_ta = q0+0.5*v_max/ramp_time*(t_a-t0)**2
                    q_tb = q_ta + v_max*(t_b - t_a)
                    q = q_tb - 0.5*v_max/ramp_time*(tf**2 - 2*tf*t + t**2 - ramp_time**2)
                    # q=0
                    
                    # q0+0.5*v_max/ramp_time*(num_ramp_points/frequency)**2 + v_max*(((points_in_segment-num_ramp_points)/frequency)-(num_ramp_points/frequency))-0.5*v_max/ramp_time*(tf**2-2*tf*t+t**2-ramp_time**2)
                else:
                    q_ta = q0+0.5*v_max/ramp_time*(t_a-t0)**2
                    q =  q_ta + v_max*(t-t_a)
                    # q=0


                # print("q",q)
                trajectory[joint, segment_start_point + i] = q

        # --------------- END STUDENT SECTION ------------------------------------

        segment_start_point += points_in_segment

    return trajectory

def checkpoint_lerp(waypoints, times):
    # q0 and q1 are 1x7 matricies 
    # f contains our constrained interpolation

    f = trajectory_trap_vel(waypoints, times)

    print(f)
    print(f.shape)

    # discrete_points = np.zeros()
    # print(discrete_points)
    # print(discrete_points.shape)


    # q0 = waypoints[0]
    # print(q0.shape)
    # print(q0)
    # q1 = waypoints[1]

    # for i in range(7):
    #     x = q0[i] * (1-f) + q1[i] * f
    #     print("SHAPE OF THIS STUFF: ",x.shape)
    #     discrete_points[i, :] = q0[i] * (1-f) + q1[i] * f

    # returns a matrix that is nx7
    return f

def _slerp(q0, q1, t):
    """Spherical Linear Interpolation between quaternions"""
    cos_theta = np.dot(q0, q1)
    if cos_theta < 0:
        q1 = -q1
        cos_theta = -cos_theta
    if cos_theta > 0.9995:
        return q0 + t*(q1 - q0)
    theta = np.arccos(cos_theta)
    sin_theta = np.sin(theta)
    s0 = np.sin((1-t)*theta) / sin_theta
    s1 = np.sin(t*theta) / sin_theta
    return s0*q0 + s1*q1
        
def _rotation_to_quaternion(R):
    """Convert rotation matrix to quaternion"""
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2,1] - R[1,2]) / S
        qy = (R[0,2] - R[2,0]) / S
        qz = (R[1,0] - R[0,1]) / S
    else:
        if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
            S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            qw = (R[2,1] - R[1,2]) / S
            qx = 0.25 * S
            qy = (R[0,1] + R[1,0]) / S
            qz = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            qw = (R[0,2] - R[2,0]) / S
            qx = (R[0,1] + R[1,0]) / S
            qy = 0.25 * S
            qz = (R[1,2] + R[2,1]) / S
        else:
            S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            qw = (R[1,0] - R[0,1]) / S
            qx = (R[0,2] + R[2,0]) / S
            qy = (R[1,2] + R[2,1]) / S
            qz = 0.25 * S
    return np.array([qw, qx, qy, qz])

def _quaternion_to_rotation(self, q):
    """Convert quaternion to rotation matrix"""
    qw, qx, qy, qz = q
    return np.array([
        [1 - 2*qy*qy - 2*qz*qz,  2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw,      1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw,      2*qy*qz + 2*qx*qw,     1 - 2*qx*qx - 2*qy*qy]
    ])

def rotation_matrix_to_euler_angles(R):
    # Roll (x-axis rotation)
    roll = np.arctan2(R[2, 1], R[2, 2])

    # Pitch (y-axis rotation)
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))

    # Yaw (z-axis rotation)
    yaw = np.arctan2(R[1, 0], R[0, 0])

    return roll, pitch, yaw