#!/usr/bin/env python3

from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
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

class ROS2WebBridge(Node):
    def __init__(self):
        super().__init__('ros2_web_bridge')
        
        # 웹에서 조인트 값을 받아 /joint_commands 토픽으로 전송
        self.publisher_ = self.create_publisher(Float64MultiArray, '/joint_commands', 10)

        # /joint_states 토픽 구독하여 변경 감지
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # WebSocket 연결 클라이언트 저장소
        self.command_clients = set()  # 조인트 명령 클라이언트
        self.log_clients = set()      # 조인트 상태 로그 클라이언트

        # 마지막으로 저장된 조인트 상태 (변경 감지용)
        self.last_joint_state = None

    async def send_log_to_clients(self, data):
        """변경된 조인트 상태를 웹 UI로 전송"""
        for ws in list(self.log_clients):
            try:
                await ws.send_text(json.dumps(data))
            except Exception as e:
                print(f"❌ WebSocket Error: {e}")
                self.log_clients.discard(ws)

    def joint_state_callback(self, msg):
        """ROS2에서 받은 조인트 상태를 웹 UI로 전송"""
        joint_data = {
            "joint1": round(msg.position[0], 3),
            "joint2": round(msg.position[1], 3),
            "joint3": round(msg.position[2], 3),
        }

        # 변경이 있을 때만 업데이트
        if self.last_joint_state is None or self.last_joint_state != joint_data:
            self.last_joint_state = joint_data
            self.get_logger().info(f"✅ Updated Joint State: {joint_data}")
            asyncio.run(self.send_log_to_clients(joint_data))

@app.get("/")
def read_root():
    return {"message": "PUMA Robot WebSocket Server Running"}

@app.websocket("/ws")
async def websocket_command_endpoint(websocket: WebSocket):
    """웹에서 조인트 명령을 수신하는 WebSocket"""
    global ros_node
    await websocket.accept()
    ros_node.command_clients.add(websocket)

    try:
        while True:
            data = await websocket.receive_text()
            parsed_data = json.loads(data)
            if isinstance(parsed_data, list) and len(parsed_data) == 3:
                msg = Float64MultiArray()
                msg.data = parsed_data
                ros_node.publisher_.publish(msg)
                print(f"📤 Published to ROS2: {msg.data}")
            else:
                print(f" Invalid data format: {parsed_data}")
    except Exception as e:
        print(f" WebSocket Error: {e}")
    finally:
        ros_node.command_clients.discard(websocket)
        print(" WebSocket Command connection closed")

@app.websocket("/ws_log")
async def websocket_log_endpoint(websocket: WebSocket):
    """웹에서 조인트 상태 로그를 수신하는 WebSocket"""
    global ros_node
    await websocket.accept()
    ros_node.log_clients.add(websocket)

    try:
        while True:
            await asyncio.sleep(1)  # 연결 유지
    except:
        pass
    finally:
        ros_node.log_clients.discard(websocket)
        print(" WebSocket Log connection closed")

def ros2_thread():
    """ROS2 실행 스레드"""
    global ros_node
    rclpy.init()
    ros_node = ROS2WebBridge()

    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)

    try:
        print(" ROS2 Web Bridge Running...")
        executor.spin()
    except Exception as e:
        print(f" ROS2 Error: {e}")
    finally:
        print(" Stopping ROS2 Node")
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    ros2_thread = threading.Thread(target=ros2_thread, daemon=True)
    ros2_thread.start()
    print(" Starting WebSocket Server on http://0.0.0.0:8005")
    uvicorn.run(app, host="0.0.0.0", port=8005, log_level="info")
