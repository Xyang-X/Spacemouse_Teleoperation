import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import asyncio
import websockets
import json
from franka_ctl import franka_spm    
import time
import threading
import numpy as np


msg={'command':'QUEST'}          #   Franka执行的任务，初始状态为休眠
def franka_ctrl():
    franka_controller = franka_spm()
    global response
    global msg
    while True:
        try:
            command = msg.get("command")
            if command == 'VELOCITY_CONTROL':
                action = msg.get("action")
                franka_controller.velocity_control(action)
            elif command == 'POSE_CONTROL':
                action = msg.get("action")
                mode = msg.get("mode")
                franka_controller.pose_control(action,mode=mode)
            elif command == 'JOINT_CONTROL':
                action = msg.get("action")
                franka_controller.joint_control(action)
            elif command == 'GRASP':
                franka_controller.grasp()
            elif command == 'RELEASE':
                franka_controller.release()
            elif command == 'STOP':
                franka_controller.velocity_control(np.array([0, 0, 0, 0, 0, 0], dtype=np.float32))  # 停止机器人
            elif command == 'QUEST':
                pass
            response = {    'status': command,
                        'ee_pose': franka_controller.Affine2list(franka_controller.ee_pose),
                        'ee_twist': franka_controller.Twist2list(franka_controller.ee_twist),
                        'elbow_vel': franka_controller.Twist2list(franka_controller.ee_twist),
                        'joint_pos': franka_controller.joint_pos.tolist(),
                        'joint_vel': franka_controller.joint_vel.tolist(),
                        }
        except Exception as e:
            print(f"Error in franka_ctrl: {e}")
            response = {'status': 'error', 'message': str(e)}
            franka_controller= franka_spm()  # 重置franka_controller
        
        
        time.sleep(0.1)  # Control loop frequency

async def handle_connection(websocket):
    try:
        async for message in websocket:
            global response
            global msg
            msg = json.loads(message)
            await websocket.send(json.dumps(response))
        

    except Exception as e:
        print(f"Connection error: {e}")
    finally:

        await websocket.close()

# 启动 WebSocket 服务器
async def start_server():
    server = await websockets.serve(handle_connection, "0.0.0.0", 8765)
    print("WebSocket服务器已启动，等待连接...")
    # await server.wait_closed()
    await asyncio.Future()  # 使服务器持续运行

# 在后台启动其他任务
def start_background_task():
    threading.Thread(target=franka_ctrl, daemon=True).start()

# 主函数
def main():
    start_background_task()
    asyncio.run(start_server())

if __name__ == "__main__":
    main()