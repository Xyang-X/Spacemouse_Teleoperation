import asyncio
import websockets
import json
import time

async def hello():
    uri = "ws://192.168.31.190:8765"  # 换成服务器局域网IP
    async with websockets.connect(uri) as websocket:
        # 构造 JSON 数据
        message = {"command": "QUEST"}
        await websocket.send(json.dumps(message))
        print("发送:", message)

        # 接收服务器返回
        response = await websocket.recv()
        ee_pose = json.loads(response).get("ee_pose", {})
        ee_pose[0][2] -= 0.1  # 修改末端执行器位置
        print("收到:", json.loads(response))

        time.sleep(1)  # 等待一秒后发送下一个消息
        print(ee_pose)
        message = {"command": "POSE_CONTROL", "action": ee_pose[0]}
        await websocket.send(json.dumps(message))
        print("发送:", message)
        
class FrankaRemoteController():
    def __init__(self,uri):
        self.uri = uri
    
    def velocity_control(self, action):
        cmd={
            'command':"VELOCITY_CONTROL",
            'action': action}
        response = asyncio.run(self.send_command(cmd))
        return response
    
    def pose_control(self, action, mode=0):
        cmd={
            'command':"POSE_CONTROL",
            'action': action,
            'mode': mode}
        response = asyncio.run(self.send_command(cmd))
        return response

    def stop(self):
        cmd={
            'command':"STOP"}
        response = asyncio.run(self.send_command(cmd))
        return response

    def quest(self):
        cmd={
            'command':"QUEST"}
        response = asyncio.run(self.send_command(cmd))
        return response
    
    def grasp(self):
        cmd={
            'command':"GRASP"}
        response = asyncio.run(self.send_command(cmd))
        return response
    
    def release(self):
        cmd = {
            'command':"RELEASE"}
        response = asyncio.run(self.send_command(cmd))
        return response

    async def send_command(self, command):
        async with websockets.connect(self.uri) as websocket:

            await websocket.send(json.dumps(command))
            print("发送:", command)
            
            response = await websocket.recv()
            print("收到:", json.loads(response))
            return response
    
    

if __name__ == "__main__":
    franka_controller = FrankaRemoteController("ws://192.168.31.190:8765")
    franka_controller.velocity_control([0, 0.05, 0.0, 0.0, 0.0, 0.0])  # Example twist input
    print(franka_controller.stop())  # 停止机器人

