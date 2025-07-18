from spacemouse import Spacemouse
import time
import numpy as np
from ur_ctl import URController

import matplotlib.pyplot as plt
import numpy as np
import time
from collections import deque

if __name__ == "__main__":


    # Initialize the Franka controller
    ur_controller = URController()
    try:
        with Spacemouse(deadzone=0.1,transmat=3) as sm:
            gripper_state = False
            while True:
                # Get the motion state from the SpaceMouse
                motion_state = sm.get_motion_state_transformed()
                if sm.is_button_pressed(0):
                    gripper_state = not gripper_state
                    if gripper_state:
                        print("Grasping...")
                        ur_controller.grasp()
                    else:
                        print("Releasing...")
                        ur_controller.release()
                rot_permit = sm.is_button_pressed(1)

                if not rot_permit:
                    motion_state[3:] = 0
                else:
                    motion_state[:3] = 0
                # Send the motion state to the Franka controller
                ur_controller.pose_control(motion_state)
                # ur_controller.velocity_control(motion_state)

                time.sleep(.1)
    except KeyboardInterrupt:
        print("Stopping...")
        ur_controller.gripper.disconnect()
        ur_controller.disconnect()
        # franka_controller.velocity_control(np.zeros(6, dtype=np.float32))  # Stop the robot
        print("Stopped.")   
