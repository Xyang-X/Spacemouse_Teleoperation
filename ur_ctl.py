from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
import time
import numpy as np
from gripper import Gripper
from scipy.spatial.transform import Rotation as R

class URController:
    def __init__(self, robot_ip="192.168.1.2", init_joints=True):
        self.robot_ip = robot_ip
        self.rtde_c = RTDEControlInterface(robot_ip,125)
        self.rtde_r = RTDEReceiveInterface(robot_ip,125)
        self.current_pose = self.rtde_r.getActualTCPPose()
        self.gripper = Gripper()
        print("Connected to UR robot at", robot_ip)
        if init_joints:
            self.rtde_c.moveJ([-1.57, -1.57, -1.57, -1.57, 1.57, 3.14])  # Move to a safe initial position

        
    def velocity_control(self, vel):
        vel[:3] *= 0.6  # Scale the velocity
        vel[3:] *= 0.5

        self.rtde_c.speedL(vel, 0.1, 1.2)  # Move with linear interpolation at 0.1 m/s
        self.target_speed = np.linalg.norm(vel[:3])  # Calculate target speed
        self.actual_speed = np.linalg.norm(self.rtde_r.getActualTCPSpeed())
        
        # print('Speed bias: ', target_speed - actual_speed)
        self.speed_bias = self.target_speed - self.actual_speed
        
    def pose_control(self, pose):
        self.current_pose = self.rtde_r.getActualTCPPose()
        trans = self.current_pose[:3] + pose[:3]*0.05
        pose[3:]*=0.2
        pose[4:]*=-1
        r0 = R.from_rotvec(self.current_pose[3:6])
        r1 = R.from_rotvec(pose[3:6])
        rot = (r0 * r1).as_rotvec()
        target_pose = np.concatenate([trans, rot])
        self.rtde_c.servoL(target_pose, 0.1, 0.1, 1/125,0.15,200)

    def grasp(self):
        """
        Grasp the object by moving the end-effector down
        """
        self.gripper.close_gripper()
        
    def release(self):
        """Release the object by moving the end-effector up
        """
        self.gripper.open_gripper()
        
    def disconnect(self):
        """Disconnect the RTDE interfaces and the gripper."""
        self.rtde_c.disconnect()
        self.rtde_r.disconnect()
        time.sleep(0.2)
        self.gripper.disconnect()
        print("Disconnected from UR robot and gripper.")