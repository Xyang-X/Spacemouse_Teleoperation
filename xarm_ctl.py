from xarm.wrapper import XArmAPI
import time
class Xarm():
    def __init__(self):
        self.arm = XArmAPI('192.168.1.221')  
        self.arm.connect()
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(5)  
        self.arm.set_state(state=0)  # Set to idle state
        self.arm.set_gripper_enable(True)
        self.pose=self.arm.get_position()  # Initialize the arm position
        print(f"Initial pose: {self.pose}")
        self.arm.move_gohome()
        time.sleep(1)
    # def move_to(self, pose):
    #     """
    #     Move the robot arm to a specified position.
    #     :param position: A list or tuple of [x, y, z, roll, pitch, yaw] in mm and degrees.
    #     :param wait: If True, wait for the motion to complete.
    #     """
    #     pose*=2
    #     print(f"Moving to pose: {pose}")
    #     pose=self.pose[1] + pose
    #     self.arm.set_position(x=pose[0],y=pose[1],z=pose[2],
    #                           roll=pose[3], pitch=pose[4], yaw=pose[5],relative=True)
    #     self.pose=self.arm.get_position()
        
    def velocity_control(self, velocity):
        velocity[:3] *= 50  # Scale the linear velocity
        velocity[3:] *= 15  # Scale the angular velocity
        print(f"Setting velocity: {velocity}")
        self.arm.vc_set_cartesian_velocity(velocity)
    def grasp(self):
        """
        Grasp the object by closing the gripper.
        """
        self.arm.set_gripper_position(0, wait=True)

    def release(self):
        """
        Release the object by opening the gripper.
        """
        self.arm.set_gripper_position(100, wait=True)

    def shutdown(self):
        """
        Shutdown the arm and close the connection.
        """
        self.arm.disconnect()
        print("XArm connection closed.")