from spacemouse import Spacemouse
import time
import numpy as np
from franka_ctl import franka_spm

if __name__ == "__main__":


    # Initialize the Franka controller
    franka_controller = franka_spm(start_grip=True)
    
    try:
        with Spacemouse(deadzone=0.1,transmat=2) as sm:
            while True:
                t1=time.time()
                # Get the motion state from the SpaceMouse
                motion_state = sm.get_motion_state_transformed()
                close = sm.is_button_pressed(0)
                open = sm.is_button_pressed(1)
                # print(motion_state, close, open)

                # Send the motion state to the Franka controller
                # motion_state[2:]=0
                franka_controller.velocity_control(motion_state)
                # franka_controller.waypoint_control(motion_state)
                
                if close:
                    print("Grasping...")
                    franka_controller.grasp()
                if open:
                    print("Releasing...")
                    franka_controller.release()
                # Sleep for a short duration to control the loop frequency
                time.sleep(.01)
                t2=time.time()
                # print(t2-t1)
    except KeyboardInterrupt:
        print("Stopping...")
       
        # franka_controller.velocity_control(np.zeros(6, dtype=np.float32))  # Stop the robot
        print("Stopped.")   
