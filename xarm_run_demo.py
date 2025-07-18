from spacemouse import Spacemouse
import time
import numpy as np
from xarm_ctl import Xarm

if __name__ == "__main__":


    # Initialize the Franka controller
    xarm_controller = Xarm()
    
    try:
        with Spacemouse(deadzone=0.1,transmat=3) as sm:
            while True:
                # Get the motion state from the SpaceMouse
                motion_state = sm.get_motion_state_transformed()
                close = sm.is_button_pressed(0)
                open = sm.is_button_pressed(1)
                # print(motion_state, close, open)

                # Send the motion state to the Franka controller
                xarm_controller.velocity_control(motion_state)
                if close:
                    print("Grasping...")
                    xarm_controller.grasp()
                if open:
                    print("Releasing...")
                    xarm_controller.release()
                # Sleep for a short duration to control the loop frequency
                time.sleep(.01)
    except KeyboardInterrupt:
        print("Stopping...")
        xarm_controller.shutdown()
        # franka_controller.velocity_control(np.zeros(6, dtype=np.float32))  # Stop the robot
        print("Stopped.")   
