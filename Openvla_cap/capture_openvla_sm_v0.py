#!/usr/bin/env python3
"""
Capture OpenVLA tele‑op collector (v7‑fix)
─────────────────────────────────────────
Quest ▶ UDP/8001 ▶ This script ▶ RTDE 30004 ▶ UR5e
                                    ├─ RealSense RGB → frames/*.jpg
                                    ├─ meta.csv (ts, tcp6, q6, grip_state, img, prompt)
                                    └─ prompt.txt (episode‑level instruction)

Bug‑fix 2025‑07‑01
------------------
* **SyntaxError** at `cv2.imwrite(...` – missing closing parenthesis.
  Now `cv2.imwrite(str(sess/"frames"/img_name), rgb,
                   [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_Q])`.
* Refined "zero‑offset" logic: first 10 frames are averaged for a stable origin.
* Added explicit `frame_idx` print every 50 frames for live feedback.

Usage
-----
```bash
python openvla_finetune/capture_openvla_v7.py --prompt "pick up the banana and put it in the basket"
```
This creates `dataset_root/session_YYYY‑MM‑DD_HH‑MM‑SS/` with images, CSV, and prompt.txt – ready for RLDS conversion.
"""

# ===== Imports =====
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import argparse, csv, socket, time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

from spacemouse import Spacemouse  # optional, for tele‑op control
from ur_ctl import URController
# ====================

# ---------- Parameters ----------
ROBOT_IP           = "192.168.1.2"
GRIPPER_IP         = "192.168.1.11"

INIT_POS           = np.array([0.0, -0.50, 0.35])
FIXED_RV           = R.from_euler("xyz", [np.pi, 0, 0]).as_rotvec()

CONTROL_HZ         = 10              # 100 ms loop
POS_EPS            = 0.002           # 2 mm dead‑band
MAX_STEP           = 0.05            # 5 cm per frame safety cap
SERVO_SPEED        = 0.30
SERVO_ACCEL        = 1.20
SERVO_TIME         = 0.20            # ≥ loop_dt for smooth blending
SERVO_LOOKAHEAD    = 0.05
SERVO_GAIN         = 300

GRIPPER_COOLDOWN   = 1.0             # s between actions

RGB_W, RGB_H       = 640, 480
JPEG_Q             = 95
SAVE_ROOT          = "dataset_root"
# --------------------------------

# ---------- RealSense helper ----------

def init_rs():
    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_stream(rs.stream.color, RGB_W, RGB_H, rs.format.bgr8, 30)
    pipe.start(cfg)
    return pipe
# --------------------------------------



# --------------------------- main ----------------------------

def main():
    # —— CLI ——
    ap = argparse.ArgumentParser(description="Capture OpenVLA trajectory with gripper & prompt support")
    ap.add_argument("--prompt", required=True, help="Language instruction for this episode")
    args = ap.parse_args()
    PROMPT = args.prompt.strip()
    print(f"▶ OpenVLA capture started – prompt: '{PROMPT}'")

    # —— Interfaces ——
    ur_controller = URController(init_joints=True)  

    cam      = init_rs()


    # —— Session folders / files ——
    sess = Path(SAVE_ROOT) / datetime.now().strftime("session_%Y-%m-%d_%H-%M-%S")
    (sess / "frames").mkdir(parents=True, exist_ok=True)
    (sess / "prompt.txt").write_text(PROMPT + "\n", encoding="utf-8")

    csv_f   = open(sess / "meta.csv", "w", newline="", encoding="utf-8")
    writer  = csv.writer(csv_f)
    writer.writerow([
        "ts", "x", "y", "z", "rx", "ry", "rz",
        *[f"q{i}" for i in range(6)], "grip_state", "img", "prompt",
    ])

    # —— Loop state ——
    loop_dt       = 1.0 / CONTROL_HZ
    pending_pos   = []              # first 10 frames for zero‑offset
    base_pos      = None
    last_pose     = ur_controller.current_pose.copy()
    gripper_state = False

    last_grip_ts  = 0.0
    last_g = last_r = 0

    frame_idx = 0
    

    try:
        with Spacemouse(deadzone=0.1,transmat=3) as sm:
            while True:
                t0 = time.time()
                motion_state = sm.get_motion_state_transformed()
                motion_state[3:5] = 0  # Disable rotation of x,y axis
                if sm.is_button_pressed(0):
                    gripper_state = not gripper_state
                    if gripper_state:
                        print("Grasping...")
                        ur_controller.grasp()
                    else:
                        print("Releasing...")
                        ur_controller.release()
                # rot_permit = sm.is_button_pressed(1)
                if sm.is_button_pressed(1):
                    break

                # if not rot_permit:
                #     motion_state[3:] = 0
                # else:
                #     motion_state[:3] = 0
                # Send the motion state to the Franka controller
                ur_controller.pose_control(motion_state)


                d_xyz = motion_state[:3] - last_pose[:3]
                dist  = np.linalg.norm(d_xyz)
                if dist > POS_EPS:
                    if dist > MAX_STEP:
                        d_xyz = d_xyz / dist * MAX_STEP
                    last_pose = motion_state.copy()


                now   = time.time()


                # 6. camera & csv
                frm = cam.poll_for_frames()
                if frm and frm.get_color_frame():
                    c   = frm.get_color_frame()
                    rgb = np.asanyarray(c.get_data())
                    ts  = c.get_timestamp() / 1e3  # ms → s
                    img_name = f"{ts:.6f}_rgb.jpg"
                    cv2.imwrite(str(sess / "frames" / img_name), rgb,
                                [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_Q])

                    tcp6 = ur_controller.current_pose
                    q6   = ur_controller.joint_positions
                    grip_state = "closed" if not gripper_state else "open"
                    writer.writerow([ts, *tcp6, *q6, grip_state, img_name, PROMPT])

                    frame_idx += 1
                    if frame_idx % 50 == 0:
                        print(f"[{frame_idx}] Δ={dist*1000:4.1f} mm → {img_name}")

                # 7. maintain loop rate
                dt = time.time() - t0
                if dt < loop_dt:
                    time.sleep(loop_dt - dt)

    except KeyboardInterrupt:
        print("\n⏹ Interrupted by user – stopping")
    finally:
        cam.stop(); csv_f.close()
        # rtde_c.servoStop(); rtde_c.disconnect(); grip.disconnect()
        ur_controller.gripper.disconnect()
        ur_controller.disconnect()
        print("✅ Session ended – data saved to", sess)


if __name__ == "__main__":
    main()
