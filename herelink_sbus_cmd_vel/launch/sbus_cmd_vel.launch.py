from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="herelink_sbus_cmd_vel",
            executable="sbus_cmd_vel_node",
            name="sbus_cmd_vel_node",
            output="screen",
            parameters=[{
                "port": "/dev/ttyACM0",
                "linear_channel": 2,
                "lateral_channel": 4,
                "angular_channel": 1,
                "center_pwm": 1524,
                "min_pwm": 1102,
                "max_pwm": 1927,
                "deadband_pwm": 30,
                "linear_direction": -1.0,
                "lateral_direction": -1.0,
                "angular_direction": -1.0,
                "max_linear": 0.5,
                "max_lateral": 0.5,
                "max_angular": 1.0,
                "debug_output": True,
            }],
        )
    ])
