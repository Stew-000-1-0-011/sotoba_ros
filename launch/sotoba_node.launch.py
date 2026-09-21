from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("sotoba_ros"), "config", "sotoba_node.yaml"]
                ),
                description="parameter file for sotoba_node",
            ),
            Node(
                package="sotoba_ros",
                executable="sotoba_node",
                name="sotoba_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
