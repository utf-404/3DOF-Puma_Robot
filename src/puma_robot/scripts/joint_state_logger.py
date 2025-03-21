#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import threading
import json
import asyncio

# FastAPI 설정
app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class JointStateLogger(Node):
    def __init__(self):
        super().__init__('joint_state_logger')

        # Joint State Subscriber
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # 웹소켓 클라이언트 목록 (로그 수신자)
        self.log_clients = set()
        
        # 마지막으로 저장된 상태 (변경 감지용)
        self.last_joint_state = None

    def joint_state_callback(self, msg):
        """ROS2 /joint_states 토픽을 구독하여 웹으로 변경된 값만 전송"""
        joint_data = {
            "joint1": round(msg.position[0], 3),
            "joint2": round(msg.position[1], 3),
            "joint3": round(msg.position[2], 3),
        }

        # 변경 감지 로직 (값이 변한 경우에만 업데이트)
        if self.last_joint_state is None or self.last_joint_state != joint_data:
            self.last_joint_state = joint_data  # 변경된 값 업데이트
            self.get_logger().info(f"✅ Updated Joint State: {joint_data}")
            asyncio.run(self.send_log_to_clients(joint_data))  # 웹 UI로 전송

    async def send_log_to_clients(self, data):
        """웹 UI로 변경된 조인트 상태 데이터 전송"""
        for ws in list(self.log_clients):
            try:
                await ws.send_text(json.dumps(data))
            except Exception as e:
                print(f"❌ WebSocket Error: {e}")
                self.log_clients.discard(ws)

    def add_log_client(self, client):
        self.log_clients.add(client)

    def remove_log_client(self, client):
        self.log_clients.discard(client)

@app.get("/")
def read_root():
    return {"message": "Joint State Logger Running"}

@app.websocket("/ws_log")
async def websocket_log_endpoint(websocket: WebSocket):
    """웹 UI에서 조인트 상태 로그를 수신하는 WebSocket"""
    global logger_node
    await websocket.accept()
    logger_node.add_log_client(websocket)

    try:
        while True:
            await asyncio.sleep(1)  # 연결 유지
    except:
        pass
    finally:
        logger_node.remove_log_client(websocket)
        print(" WebSocket Log connection closed")

def ros2_thread():
    """ROS2 실행 스레드"""
    global logger_node
    rclpy.init()
    logger_node = JointStateLogger()

    executor = MultiThreadedExecutor()
    executor.add_node(logger_node)

    try:
        print(" ROS2 Joint State Logger Running...")
        executor.spin()
    except Exception as e:
        print(f" ROS2 Error: {e}")
    finally:
        print(" Stopping ROS2 Node")
        logger_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    ros2_thread = threading.Thread(target=ros2_thread, daemon=True)
    ros2_thread.start()
    print("🚀 Starting WebSocket Server on http://0.0.0.0:8010")
    uvicorn.run(app, host="0.0.0.0", port=8010, log_level="info")
