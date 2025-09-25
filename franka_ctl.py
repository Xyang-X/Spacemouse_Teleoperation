from franky import *
import numpy as np
from collections import defaultdict
import time
from scipy.spatial.transform import Rotation as R, Slerp

class franka_spm():
    def __init__(self,translation_inter_scale=0.2,rotation_inter_scale=0.8,start_grip=False):
        self.robot = Robot("172.16.0.2")
        if start_grip:
            self.gripper = Gripper("172.16.0.2")
        self.cb()  # Initialize the robot state
        self.robot.relative_dynamics_factor = RelativeDynamicsFactor(0.05, 0.1, 0.15)
        
        self.translation_inter_scale=translation_inter_scale
        self.rotation_inter_scale=rotation_inter_scale
        eular=R.from_quat(self.ee_pose.quaternion).as_euler('xyz', degrees=True)        
        # print(eular)
        print(self.ee_pose)

        
    def velocity_control(self, twist):
        """
        twist: [vx, vy, vz, wx, wy, wz]
        """
        self.cb()
        twist=np.array(twist,dtype=np.float32)
        # Transform the velocity to the robot's coordinate system
        linear_velocity= twist[:3]*self.translation_inter_scale
        angular_velocity= twist[3:]*self.rotation_inter_scale
        m_cv=CartesianVelocityMotion(Twist(linear_velocity, angular_velocity), relative_dynamics_factor=0.2)

        self.robot.move(m_cv, asynchronous=True)    


    def cb(self):
        """
        Callback function to be called periodically
        """
        self.state = self.robot.state
        self.cartesian_state = self.robot.current_cartesian_state
        self.robot_pose = self.cartesian_state.pose  # Contains end-effector pose and elbow position
        self.ee_pose = self.robot_pose.end_effector_pose
        robot_velocity = self.cartesian_state.velocity  # Contains end-effector twist and elbow velocity
        self.ee_twist = robot_velocity.end_effector_twist
        self.elbow_vel = robot_velocity.elbow_velocity
        joint_state = self.robot.current_joint_state
        self.joint_pos = joint_state.position
        self.joint_vel = joint_state.velocity
    
    # def cb(robot_state: RobotState, time_step: Duration, rel_time: Duration, abs_time: Duration, control_signal: JointPositions):
        # print(f"At time {abs_time}, the target joint positions were {control_signal.q}")
        
    def pose_control(self, target_pose,mode=0):
        """
        Move the robot to the target position

        mode: 0 for 6D pose input , 1 for 3D position and 4D quaternion input
        """
        if mode==0:
            r=R.from_euler('xyz', target_pose[3:])
            quat= r.as_quat()
            m_cp=CartesianMotion(Affine(target_pose[:3],quat),relative_dynamics_factor=0.2)
        elif mode==1:
            m_cp=CartesianMotion(Affine(target_pose[:3],target_pose[3:]),relative_dynamics_factor=0.2)
        self.robot.move(m_cp)  

    def joint_control(self, target_joint):
        """
        Move the robot to the target joint position
        """
        m_jp=JointMotion(target_joint,relative_dynamics_factor=0.2)
        self.robot.move(m_jp, asynchronous=True)  

    def grasp(self):
        """
        Grasp the object by moving the end-effector down
        """
        speed = 0.05  # [m/s]
        force = 10.0  # [N]
        success = self.gripper.grasp(0.0, speed, force, epsilon_outer=1.0)
        return success
    
    def release(self):
        """
        Release the object by moving the end-effector up
        """
        speed = 0.05  # [m/s]
        self.gripper.open(speed)
    
    def Affine2list(self,affine):
        """
        Convert Affine to list
        """
        translation = affine.translation
        rotation = affine.quaternion
        # return [translation[0], translation[1], translation[2], rotation[0], rotation[1], rotation[2], rotation[3]]
        return [translation.tolist(),translation.tolist()]
    
    def Twist2list(self,twist):
        """
        Convert Twist to list
        """
        linear = twist.linear
        angular = twist.angular
        return [linear[0], linear[1], linear[2], angular[0], angular[1], angular[2]]
        
if __name__ == "__main__":
    franka_spm_instance = franka_spm()

    # franka_spm_instance.velocity_control(np.array([0, 0.1, 0.0, 0.0, 0.0, 0.0], dtype=np.float32))  # Example twist input
    # time.sleep(1)  # Allow some time for the motion to execute
    # franka_spm_instance.velocity_control(np.array([0.0, 0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32))  # Example twist input
    # # Example twist input
    # with RobotWebSession("172.16.0.2", "franka", "franka123") as robot_web_session:
    # # First take control
    #     robot_web_session.take_control(wait_timeout=10.0)
 
    #     # Unlock the brakes 
    #     # robot_web_session.unlock_brakes()
    
    #     # # Enable the FCI
    #     # robot_web_session.enable_fci()
        
    #     # Create a franky.Robot instance and do whatever you want
        
        
    
    #     # Disable the FCI
    #     # robot_web_session.disable_fci()
    
    #     # Lock brakes
    #     robot_web_session.lock_brakes()

    
