import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import asyncio
import websockets
import json
from franka_ctl import franka_spm    
import time
import threading


command='QUEST'          #   Franka执行的任务，初始状态为休眠
def franka_ctrl():
    franka_controller = franka_spm()
    global command
    global response
    global action
    global msg
    while True:
        try:
            if msg.get("command") == 'VELOCITY_CONTROL':
                action = msg.get("action")
                franka_controller.velocity_control(action)
            elif msg.get("command") == 'POSE_CONTROL':
                action = msg.get("action")
                mode = msg.get("mode")
                franka_controller.pose_control(action,mode=mode)
            elif msg.get("command") == 'GRASP':
                franka_controller.grasp()
            elif msg.get("command") == 'RELEASE':
                franka_controller.release()
            elif msg.get("command") == 'STOP':
                franka_controller.velocity_control([0, 0, 0, 0, 0, 0])  # 停止机器人
            elif msg.get("command") == 'QUEST':
                pass
            response = {    'status': command,
                        'ee_pose': franka_controller.Affine2list(franka_controller.ee_pose),
                        'ee_twist': franka_controller.Twist2list(franka_controller.ee_twist),
                        'elbow_vel': franka_controller.Twist2list(franka_controller.ee_twist)}
        except Exception as e:
            print(f"Error in franka_ctrl: {e}")
            response = {'status': 'error', 'message': str(e)}
        
        
        time.sleep(0.1)  # Control loop frequency

async def handle_connection(websocket):
    try:
        async for message in websocket:
            global command
            global response
            global action
            global msg
            msg = json.loads(message)
            # command = msg.get("command")
            # if command=='VELOCITY_CONTROL' or command=='POSE_CONTROL': 
            #     action = msg.get("action")
            #     await websocket.send(json.dumps(response))
            # elif command=='GRASP' or command=='RELEASE':

            #     await websocket.send(json.dumps(response))
            # elif command=='QUEST':
            #     await websocket.send(json.dumps(response))
            await websocket.send(json.dumps(response))
        
            # 回复上位机
            # asyncio.run(websocket.send(json.dumps(response)))
    except Exception as e:
        print(f"Connection error: {e}")
    finally:
        asyncio.run(websocket.close())

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