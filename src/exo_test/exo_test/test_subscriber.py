import re
import rclpy
from rclpy.node import Node

from exo_msgs.msg import JointCommand


class JointCommandSubscriber(Node):
    def __init__(self):
        super().__init__("joint_command_test_subscriber")

        self.sub = self.create_subscription(
            JointCommand,
            "joint_command",
            self.cb,
            10
        )

        self.count = 0
        self.last_seq = None
        self.dropped = 0
        self.received = 0

    def cb(self, msg: JointCommand):
        now = self.get_clock().now()
        sent_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        now_ns = now.nanoseconds
        latency_ms = (now_ns - sent_ns) / 1e6

        m = re.match(r"exo_seq:(\d+)", msg.header.frame_id)
        if not m:
            return  # or log once: bad frame_id format

        seq = int(m.group(1))
        if self.last_seq is not None and seq != self.last_seq + 1:
            self.dropped += (seq - self.last_seq - 1)

        self.last_seq = seq
        self.received += 1

        self.count += 1
        if self.count % 20 == 0:  # log every 20 msgs (~5 logs/sec at 100Hz)
            self.get_logger().info(
                f"Received #{self.count} latency={latency_ms:.3f} ms "
                f"names={list(msg.name)} effort0={msg.effort[0]:.2f} "
                f"messages dropped={self.dropped}"
            )


def main():
    rclpy.init()
    node = JointCommandSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
