import math
import rclpy
from rclpy.node import Node

from exo_msgs.msg import JointCommand


class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__("joint_command_test_publisher")

        self.pub = self.create_publisher(JointCommand, "joint_command", 10)

        self.names = ["hip_left", "hip_right", "knee_left", "knee_right"]
        self.start_sec = self.get_clock().now().nanoseconds * 1e-9

        self.timer = self.create_timer(0.01, self.tick)  # 100 Hz
        self.seq = 0

    def tick(self):
        now = self.get_clock().now()
        t = (now.nanoseconds * 1e-9) - self.start_sec

        msg = JointCommand()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = f"exo_seq:{self.seq}"
        self.seq += 1
        msg.name = self.names

        # Dummy torques: sine wave, phase-shifted per joint
        msg.effort = [
            10.0 * math.sin(2.0 * math.pi * 0.5 * t + 0.0),
            10.0 * math.sin(2.0 * math.pi * 0.5 * t + 0.5),
            10.0 * math.sin(2.0 * math.pi * 0.5 * t + 1.0),
            10.0 * math.sin(2.0 * math.pi * 0.5 * t + 1.5),
        ]

        self.pub.publish(msg)

        # Print timestamp occasionally so you don’t spam the terminal
        if int(t * 100) % 50 == 0:  # ~2 Hz print
            self.get_logger().info(f"Published @ {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}")


def main():
    rclpy.init()
    node = JointCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
