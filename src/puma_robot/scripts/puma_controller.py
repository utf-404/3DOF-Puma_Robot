#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class PumaController(Node):
    def __init__(self):
        super().__init__('puma_controller')

        # /joint_commands 토픽 구독 (웹에서 값 수신)
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/joint_commands',
            self.command_callback,
            10)
        
        # /joint_states 토픽 퍼블리시 (URDF 반영)
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        # 초기 조인트 상태 설정 (초기 상태 유지)
        self.current_joint_state = JointState()
        self.current_joint_state.position = [0.0, 0.0, 0.0]
        self.current_joint_state.name = ["joint1", "joint2", "joint3"]

        # 100ms마다 현재 상태를 퍼블리시 (RViz2 반영)
        self.timer = self.create_timer(1.0, self.publish_joint_states)

    def command_callback(self, msg):
        """ 웹에서 조인트 값을 받으면 즉시 반영 """
        if len(msg.data) == 3:
            self.get_logger().info(f"✅ Received Joint Command: {msg.data}")
            self.current_joint_state.position = list(msg.data)  # 새로운 값 적용
        else:
            self.get_logger().warn(f"❌ Invalid joint command received: {msg.data}")

    def publish_joint_states(self):
        """ 현재 조인트 상태를 지속적으로 퍼블리시하여 RViz2에 반영 """
        self.current_joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.current_joint_state)
        self.get_logger().info(f"Publishing Joint States: {self.current_joint_state.position}")

def main(args=None):
    rclpy.init(args=args)
    node = PumaController()

    try:
        rclpy.spin(node)  # 지속 실행하여 상태 유지
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
