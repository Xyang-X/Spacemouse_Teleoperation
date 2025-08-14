from franky import *
import numpy as np
from collections import defaultdict
import time
from scipy.spatial.transform import Rotation as R, Slerp

class franka_spm():
    def __init__(self,translation_inter_scale=0.3,rotation_inter_scale=0.8):
        self.robot = Robot("172.16.0.2")
        # self.gripper = Gripper("172.16.0.2")
        self.state = self.robot.state
        self.cartesian_state = self.robot.current_cartesian_state
        self.robot_pose = self.cartesian_state.pose  # Contains end-effector pose and elbow position
        self.ee_pose = self.robot_pose.end_effector_pose
        self.robot.relative_dynamics_factor = RelativeDynamicsFactor(0.05, 0.1, 0.15)
        joint_state = self.robot.current_joint_state
        joint_pos = joint_state.position
        self.translation_inter_scale=translation_inter_scale
        self.rotation_inter_scale=rotation_inter_scale
        eular=R.from_quat(self.ee_pose.quaternion).as_euler('xyz', degrees=True)        
        # print(eular)
        print(self.ee_pose)
        print(f"Initial joint positions: {joint_pos}")
        
    def velocity_control(self, twist):
        """
        twist: [vx, vy, vz, wx, wy, wz]
        """
        self.cb1()
        # Transform the velocity to the robot's coordinate system
        linear_velocity= twist[:3]
        angular_velocity= twist[3:]
        m_cv=CartesianVelocityMotion(Twist(linear_velocity, angular_velocity), relative_dynamics_factor=0.2)
        # m_cv.register_callback(self.cb)  # Register the callback function to be called periodically
        # self.state = self.robot.state
        # self.cartesian_state = self.robot.current_cartesian_state
        # self.robot_pose = self.cartesian_state.pose  # Contains end-effector pose and elbow position
        # self.ee_pose = self.robot_pose.end_effector_pose
        # print(twist)
        # print(self.ee_pose)
        self.robot.move(m_cv, asynchronous=True)    
        
    def waypoint_control(self, pose):
        """
        pose: [x, y, z, roll, pitch, yaw]
        """
        if all(x<0.1 for x in pose):
            return
        self.cb1()
        # Transform the pose to the robot's coordinate system
        pose[:3] *= self.translation_inter_scale  # Scale translation
        pose[3:] *= 0  # Scale rotation
        currr_pose=np.concatenate([self.ee_pose.translation,R.from_quat(self.ee_pose.quaternion).as_rotvec()])
        pose_inter=self._generate_interpolated_poses(currr_pose,pose+currr_pose,3)
        pose_commands=[]
        print('Command:')
        for pose in pose_inter:
            quat=R.from_rotvec(pose[3:]).as_quat()
            pose_commands.append(CartesianWaypoint(Affine(pose[:3],quat)))
            print(pose)

        m_wp=CartesianWaypointMotion(pose_commands,relative_dynamics_factor=0.2)
        time.sleep(1)  # Ensure the robot state is updated before moving
        # m_wp.register_callback(self.cb)  # Register the callback function to be called periodically
        self.robot.move(m_wp, asynchronous=True)
    def _generate_interpolated_poses(self, start_pose: np.ndarray, end_pose: np.ndarray, num_points: int) -> np.ndarray:
        """
        在两个6D姿态（位置 + 旋转向量）之间生成插值轨迹。
        
        参数：
            start_pose: shape=(6,), 初始姿态
            end_pose: shape=(6,), 目标姿态
            num_points: int，插值点数（不含首尾）

        返回：
            poses: shape=(num_points + 2, 6)，包含起点、插值点和终点的轨迹
        """
        assert start_pose.shape == (6,)
        assert end_pose.shape == (6,)
        assert num_points >= 0

        # 拆分位置和旋转
        pos_start, pos_end = start_pose[:3], end_pose[:3]
        rot_start, rot_end = R.from_rotvec(start_pose[3:]), R.from_rotvec(end_pose[3:])

        # 创建时间轴和插值器
        key_times = [0, 1]
        interp_times = np.linspace(0, 1, num_points + 2)  # 包括首尾
        slerp = Slerp(key_times, R.concatenate([rot_start, rot_end]))

        # 插值
        positions = np.outer(1 - interp_times, pos_start) + np.outer(interp_times, pos_end)
        rotations = slerp(interp_times).as_rotvec()

        # 合并结果
        poses = np.hstack((positions, rotations))
        return poses


    def cb1(self):
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
        
        # print(self.ee_pose)   

    def cb(robot_state: RobotState, time_step: Duration, rel_time: Duration, abs_time: Duration, control_signal: JointPositions):
        print(f"At time {abs_time}, the target joint positions were {control_signal.q}")
 
 
    def pose_control(self, target_pose  ):
        """
        Move the robot to the target position
        """
        r=R.from_euler('xyz', target_pose[3:])
        quat= r.as_quat()
        print(quat)
        m_cp=CartesianMotion(Affine(target_pose[:3],quat),relative_dynamics_factor=0.2)
        self.robot.move(m_cp)    

    def grasp(self):
        """
        Grasp the object by moving the end-effector down
        """
        speed = 0.02  # [m/s]
        force = 10.0  # [N]
        success = self.gripper.grasp(0.0, speed, force, epsilon_outer=1.0)
        return success
    
    def release(self):
        """
        Release the object by moving the end-effector up
        """
        speed = 0.02  # [m/s]
        self.gripper.open(speed)
    
        
if __name__ == "__main__":
    franka_spm_instance = franka_spm()
    print(franka_spm_instance.ee_pose.translation)
    franka_spm_instance.pose_control(np.array([0.45,0,0.1,np.pi,0,0]))


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

    
