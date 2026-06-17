from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    limo_ultra_base_share = get_package_share_directory("limo_ultra_base")
    camera_autoview_share = get_package_share_directory("camera_autoview")

    declared_arguments = [
        DeclareLaunchArgument(
            "base_port_name",
            default_value="ttyTHS1",
            description="Serial port name for the LIMO Ultra base driver.",
        ),
        DeclareLaunchArgument(
            "odom_frame",
            default_value="odom",
            description="Odometry frame id for the base driver.",
        ),
        DeclareLaunchArgument(
            "base_frame",
            default_value="base_link",
            description="Base frame id for the base driver.",
        ),
        DeclareLaunchArgument(
            "odom_topic_name",
            default_value="/odom",
            description="Odometry topic name published by the base driver.",
        ),
        DeclareLaunchArgument(
            "freq",
            default_value="50.0",
            description="Base driver loop frequency.",
        ),
        DeclareLaunchArgument(
            "herelink_port",
            default_value="/dev/ttyACM0",
            description="Serial port used by the HereLink S.Bus bridge.",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/cmd_vel",
            description="cmd_vel topic published by HereLink and consumed by the base.",
        ),
        DeclareLaunchArgument(
            "max_linear",
            default_value="0.5",
            description="Maximum forward/backward speed in m/s.",
        ),
        DeclareLaunchArgument(
            "max_lateral",
            default_value="0.5",
            description="Maximum lateral speed in m/s.",
        ),
        DeclareLaunchArgument(
            "max_angular",
            default_value="1.0",
            description="Maximum yaw rate in rad/s.",
        ),
        DeclareLaunchArgument(
            "debug_output",
            default_value="false",
            description="Enable debug output from the HereLink node.",
        ),
        DeclareLaunchArgument(
            "enable_camera",
            default_value="true",
            description="Launch the fullscreen camera viewer together with driving nodes.",
        ),
        DeclareLaunchArgument(
            "camera_name",
            default_value="camera",
            description="Camera namespace passed to camera_autoview.",
        ),
        DeclareLaunchArgument(
            "color_width",
            default_value="1280",
            description="Camera color stream width.",
        ),
        DeclareLaunchArgument(
            "color_height",
            default_value="720",
            description="Camera color stream height.",
        ),
        DeclareLaunchArgument(
            "color_fps",
            default_value="30",
            description="Camera color stream FPS.",
        ),
        DeclareLaunchArgument(
            "color_format",
            default_value="MJPG",
            description="Camera color stream format.",
        ),
    ]

    limo_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [limo_ultra_base_share, "/launch/limo_ultra_base.launch.py"]
        ),
        launch_arguments={
            "port_name": LaunchConfiguration("base_port_name"),
            "odom_frame": LaunchConfiguration("odom_frame"),
            "base_frame": LaunchConfiguration("base_frame"),
            "odom_topic_name": LaunchConfiguration("odom_topic_name"),
            "freq": LaunchConfiguration("freq"),
        }.items(),
    )

    herelink_cmd_vel = Node(
        package="herelink_sbus_cmd_vel",
        executable="sbus_cmd_vel_node",
        name="sbus_cmd_vel_node",
        output="screen",
        parameters=[
            {
                "port": LaunchConfiguration("herelink_port"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
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
                "max_linear": ParameterValue(
                    LaunchConfiguration("max_linear"), value_type=float
                ),
                "max_lateral": ParameterValue(
                    LaunchConfiguration("max_lateral"), value_type=float
                ),
                "max_angular": ParameterValue(
                    LaunchConfiguration("max_angular"), value_type=float
                ),
                "debug_output": ParameterValue(
                    LaunchConfiguration("debug_output"), value_type=bool
                ),
            }
        ],
    )

    camera_view = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [camera_autoview_share, "/launch/camera_autoview.launch.py"]
        ),
        condition=IfCondition(LaunchConfiguration("enable_camera")),
        launch_arguments={
            "camera_name": LaunchConfiguration("camera_name"),
            "color_width": LaunchConfiguration("color_width"),
            "color_height": LaunchConfiguration("color_height"),
            "color_fps": LaunchConfiguration("color_fps"),
            "color_format": LaunchConfiguration("color_format"),
            "fullscreen": "true",
            "maximize_window": "false",
            "close_on_click": "true",
            "fill_mode": "crop",
        }.items(),
    )

    return LaunchDescription(declared_arguments + [limo_base, herelink_cmd_vel, camera_view])
