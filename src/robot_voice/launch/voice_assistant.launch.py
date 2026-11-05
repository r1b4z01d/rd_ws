from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "param_file",
            default_value=PathJoinSubstitution([FindPackageShare("robot_voice"), "config", "voice_assistant.yaml"]),
            description="YAML parameters for voice_assistant_node.",
        ),
        Node(
            package="robot_voice",
            executable="voice_assistant_node",
            name="robot_voice",
            output="screen",
            parameters=[LaunchConfiguration("param_file")],
        ),
    ])
