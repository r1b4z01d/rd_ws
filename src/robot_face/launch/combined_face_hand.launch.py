from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _as_bool(text: str):
    return text.lower() in ("1", "true", "yes", "on")


def _launch_setup(context):
    param_file = LaunchConfiguration("param_file").perform(context)
    img_override = LaunchConfiguration("image_topic").perform(context)
    hand_override = LaunchConfiguration("hand_in_eyes").perform(context)
    mirror_override = LaunchConfiguration("mirror").perform(context)

    params = [param_file]
    # Only override if user provided a non-empty value
    if img_override:
        params.append({"image_topic": img_override})
    if hand_override:
        params.append({"hand_in_eyes": ParameterValue(_as_bool(hand_override), value_type=bool)})
    if mirror_override:
        params.append({"mirror": ParameterValue(_as_bool(mirror_override), value_type=bool)})

    return [
        Node(
            package="robot_face",
            executable="combined_face_hand_node",
            name="combined_face_hand",
            output="screen",
            parameters=params,
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "param_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_face"), "config", "combined_face_hand_defaults.yaml"]
                ),
                description="YAML file with parameters for combined_face_hand_node.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="",
                description="Optional override: image topic to subscribe to.",
            ),
            DeclareLaunchArgument(
                "hand_in_eyes",
                default_value="",
                description="Optional override: set true to draw hand landmarks in the eyes window.",
            ),
            DeclareLaunchArgument(
                "mirror",
                default_value="",
                description="Optional override: set true to mirror the hand overlay (selfie style).",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
