from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory("fish_bringup"))
    mock_config = share / "config" / "mock.yaml"
    recording_config = share / "config" / "recording.yaml"
    qos_config = share / "config" / "qos.yaml"
    bag_root = LaunchConfiguration("bag_root")

    return LaunchDescription([
        DeclareLaunchArgument(
            "bag_root",
            default_value="bags",
            description="Root directory for per-run rosbag2 sessions.",
        ),
        Node(
            package="fish_control",
            executable="controller_node",
            name="controller",
            namespace="fish",
            parameters=[str(mock_config)],
            output="screen",
        ),
        Node(
            package="fish_tools",
            executable="experiment_manager",
            name="experiment_manager",
            namespace="fish",
            parameters=[
                str(recording_config),
                {
                    "bag_root": bag_root,
                    "qos_override_file": str(qos_config),
                },
            ],
            output="screen",
        ),
    ])
