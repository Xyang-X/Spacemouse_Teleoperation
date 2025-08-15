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
        ee_pose[2] += 0.05  # 修改末端执行器位置
        print("收到:", json.loads(response))

        time.sleep(1)  # 等待一秒后发送下一个消息
        message = {"command": "POSE_CONTROL", "action": ee_pose}
        await websocket.send(json.dumps(message))
        print("发送:", message)
        

if __name__ == "__main__":
    asyncio.run(hello())
