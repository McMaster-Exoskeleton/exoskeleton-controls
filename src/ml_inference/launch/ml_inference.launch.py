from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ml_inference",
            executable="ml_inference_node",
            name="ml_inference",
            output="screen",
            parameters=[
                {"timer_period": 0.01},
                {"kp": 10.0},
                {"mode": "sine"}, # Can change to constant or sine
            ]
        )
    ])