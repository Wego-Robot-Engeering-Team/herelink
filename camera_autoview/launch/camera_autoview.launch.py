import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    orbbec_launch = os.path.join(
        get_package_share_directory("orbbec_camera"),
        "launch",
        "ob_camera.launch.py",
    )

    return LaunchDescription([
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("color_width", default_value="1280"),
        DeclareLaunchArgument("color_height", default_value="720"),
        DeclareLaunchArgument("color_fps", default_value="30"),
        DeclareLaunchArgument("color_format", default_value="MJPG"),
        DeclareLaunchArgument("fullscreen", default_value="true"),
        DeclareLaunchArgument("maximize_window", default_value="false"),
        DeclareLaunchArgument("close_on_click", default_value="true"),
        DeclareLaunchArgument("fill_mode", default_value="crop"),
        DeclareLaunchArgument("window_width", default_value="1280"),
        DeclareLaunchArgument("window_height", default_value="720"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(orbbec_launch),
            launch_arguments={
                "camera_name": LaunchConfiguration("camera_name"),
                "enable_depth": "false",
                "enable_ir": "false",
                "enable_point_cloud": "false",
                "enable_colored_point_cloud": "false",
                "color_width": LaunchConfiguration("color_width"),
                "color_height": LaunchConfiguration("color_height"),
                "color_fps": LaunchConfiguration("color_fps"),
                "color_format": LaunchConfiguration("color_format"),
            }.items(),
        ),
        Node(
            package="camera_autoview",
            executable="camera_fullscreen_viewer",
            name="camera_fullscreen_viewer",
            output="screen",
            parameters=[{
                "image_topic": ["/", LaunchConfiguration("camera_name"), "/color/image_raw"],
                "fullscreen": LaunchConfiguration("fullscreen"),
                "maximize_window": LaunchConfiguration("maximize_window"),
                "close_on_click": LaunchConfiguration("close_on_click"),
                "fill_mode": LaunchConfiguration("fill_mode"),
                "window_width": LaunchConfiguration("window_width"),
                "window_height": LaunchConfiguration("window_height"),
            }],
        ),
    ])
