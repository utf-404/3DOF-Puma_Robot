#!/usr/bin/env python3

from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray
import uvicorn
import threading
import json
import asyncio

app = FastAPI()


app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class ROS2Bridge(Node):
    def __init__(self):
        super().__init__('ros2_bridge')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self._clients = set()  
        self.loop = asyncio.new_event_loop()

    async def send_to_clients(self, data):
        """웹 UI로 데이터 전송"""
        for ws in list(self._clients):
            try:
                await ws.send_text(json.dumps(data))
            except Exception as e:
                print(f" Error sending data to client: {e}")
                self._clients.discard(ws)

    def add_client(self, client):
        self._clients.add(client)

    def remove_client(self, client):
        self._clients.discard(client)  

@app.get("/")
def read_root():
    return {"message": "PUMA Robot WebSocket Server Running"}

async def websocket_endpoint(websocket: WebSocket):
    global ros_node
    await websocket.accept()
    ros_node.add_client(websocket)

    try:
        while True:
            data = await websocket.receive_text()
            parsed_data = json.loads(data)
            if isinstance(parsed_data, list) and len(parsed_data) == 3:
                msg = Float64MultiArray()
                msg.data = parsed_data
                ros_node.publisher_.publish(msg)
                print(f" Published to ROS2: {msg.data}")
            else:
                print(f" Invalid data format: {parsed_data}")
    except Exception as e:
        print(f" WebSocket Error: {e}")
    finally:
        ros_node.remove_client(websocket)
        print(" WebSocket connection closed")

app.add_api_websocket_route("/ws", websocket_endpoint)

def ros2_thread():
    """ROS2 실행 스레드"""
    global ros_node
    rclpy.init()
    ros_node = ROS2Bridge()

    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)

    try:
        print(" ROS2 Executor Running...")
        executor.spin()
    except Exception as e:
        print(f" ROS2 Executor Error: {e}")
    finally:
        print(" Stopping ROS2 Node")
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    ros2_thread = threading.Thread(target=ros2_thread, daemon=True)
    ros2_thread.start()
    print(" Starting WebSocket Server on http://0.0.0.0:8005")
    uvicorn.run(app, host="0.0.0.0", port=8005, log_level="info")

