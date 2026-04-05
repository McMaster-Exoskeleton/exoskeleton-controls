import time
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState # standard ROS2 package
from exo_msgs.msg import JointCommand # custom package

class MLInferenceNode(Node):
    def __init__(self):
        super().__init__("ml_inference_node")

        self.declare_parameter("timer_period", 0.01) # 100 Hz
        self.declare_parameter("kp", 10.0)
        self.declare_parameter("mode", "proportional")
        # modes: constant, proportional, sine

        self.timer_period = self.get_parameter("timer_period").value
        self.kp = self.get_parameter("kp").value
        self.mode = self.get_parameter("mode").value

        # ROS Interfaces
        self.sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_cb,
            10
        )

        self.pub = self.create_publisher(
            JointCommand,
            "/joint_commands",
            10
        )

        self.timer = self.create_timer(self.timer_period, self.timer_cb)

        # State
        self.last_joint_state = None

        # Timing stats
        self.exec_times = []
        self.dropped_frames = 0
        self.expected_period = self.timer_period

        self.last_timer_time = None

        # Rate tracking
        self.loop_count = 0
        self.start_time = time.time()

        self.get_logger().info("ML Inference Node started")

    # Subscriber
    def joint_state_cb(self, msg: JointState):
        self.last_joint_state = msg
        
    # Timer (100 Hz loop)
    def timer_cb(self):
        t_start = time.perf_counter()

        now = time.time()

        # Check for dropped frames
        if self.last_timer_time is not None:
            dt = now - self.last_timer_time
            if dt > self.expected_period * 1.5: # Allow some jitter
                self.dropped_frames += 1
        
        self.last_timer_time = now

        # If there's no data yet, skip
        if self.last_joint_state is None:
            return

        js = self.last_joint_state

        # Dummy ML logic
        effort = []

        if self.mode == "constant":
            effort = [5.0 for _ in js.position]

        elif self.mode == "proportional":
            # tau = -Kp * theta
            effort = [-self.kp * pos for pos in js.position]
        
        elif self.mode == "sine":
            t = time.time() - self.start_time
            effort = [
                5.0 * math.sin(2.0 * math.pi * 0.5 * t + i) # 0.5 Hz sine wave
                for i in range(len(js.position))
            ]
        
        else:
            self.get_logger().warn(f"Unknown mode: {self.mode}")
            effort = [0.0 for _ in js.position]

        # Publish command
        msg = JointCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ml_inference"
        
        msg.name = list(js.name)
        msg.effort = effort

        self.pub.publish(msg)

        # Timing
        exec_time = (time.perf_counter() - t_start) * 1000.0 # ms
        self.exec_times.append(exec_time)

        self.loop_count += 1

        # Print stats every second
        if time.time() - self.start_time >= 1.0:
            self.print_stats()
            self.reset_stats()

            
    # Diagnostics
    def print_stats(self):
        if len(self.exec_times) == 0:
            return
        
        avg_exec = sum(self.exec_times) / len(self.exec_times)
        max_exec = max(self.exec_times)

        elapsed = time.time() - self.start_time
        rate = self.loop_count / elapsed if elapsed > 0 else 0.0

        self.get_logger().info(
            f"ML Inference Stats (1s): "
            f"Rate ={rate:.1f} Hz, "
            f"Avg Exec Time ={avg_exec:.2f} ms, "
            f"Max Exec Time ={max_exec:.2f} ms, "
            f"Dropped Frames ={self.dropped_frames}"
        )

    def reset_stats(self):
        self.exec_times = []
        self.loop_count = 0
        self.start_time = time.time()
        self.dropped_frames = 0

    
def main(args=None):
    rclpy.init(args=args)

    node = MLInferenceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.shutdown()