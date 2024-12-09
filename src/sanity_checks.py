import numpy as np
import utils
from motion_planner import TrajectoryFollower
from frankapy import FrankaArm
import time
from robot_config import RobotConfig
from robot import Robot
import paths
import test_paths
from autolab_core import RigidTransform

config = RobotConfig()

home_joints = np.array(config.HOME_JOINTS)
rob = Robot()
fa = FrankaArm()
class WorkspaceCalibrator:
    def __init__(self):
        self.fa = FrankaArm()
        self.duration = 10
        
    def calibrate_pen_holders(self):
        print("\nStarting calibration...")
        print("Moving to home position...")
        self.fa.reset_joints()
        

        input(f"Press enter to make joints movable")
        print(f"Move robot above a pen, the position will be printed out after {self.duration} seconds")
        self.fa.run_guide_mode(duration=self.duration)  # Allow manual positioning
        
        # Record position
        joints = self.fa.get_joints()
        current_pose = self.fa.get_pose()
        c_pos = np.eye(4)
        c_pos[:3,:3] = current_pose.rotation
        c_pos[:3,3] = current_pose.translation

        calculated_pose = rob.forward_kinematics(joints)[:,:,-1]
        print("\n DIFFERENCE: \n",calculated_pose-c_pos)
       
        calculated_joints = rob._inverse_kinematics(c_pos, home_joints)

        print("\nDIFFERENCE: ",calculated_joints-joints)

        print()
        return current_pose


if __name__ == '__main__':
    
    tf = TrajectoryFollower()
    
    fa.reset_joints()
    fa.open_gripper()
    c = WorkspaceCalibrator()
    c.calibrate_pen_holders()