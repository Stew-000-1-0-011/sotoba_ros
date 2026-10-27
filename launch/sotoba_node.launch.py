from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")
    fake_scan = LaunchConfiguration("fake_scan")
    predictor = LaunchConfiguration("predictor")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                description="parameter file for sotoba_node and persistence_predictor (required: the nodes have no defaults)",
            ),
            DeclareLaunchArgument(
                "fake_scan",
                default_value="false",
                description="also launch fake_scan_publisher (synthetic /scan from objects.cpp)",
            ),
            DeclareLaunchArgument(
                "predictor",
                default_value="true",
                description=(
                    "launch persistence_predictor to feed sotoba_node's prior. "
                    "set false when another node (e.g. a state estimator) publishes it"
                ),
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="launch RViz2 with the bundled config",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("sotoba_ros"), "config", "sotoba.rviz"]
                ),
                description="RViz2 config file",
            ),
            Node(
                package="sotoba_ros",
                executable="sotoba_node",
                name="sotoba_node",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="sotoba_ros",
                executable="persistence_predictor",
                name="persistence_predictor",
                output="screen",
                parameters=[params_file],
                remappings=[
                    ("posterior_beliefs", "/sotoba_node/posterior_beliefs"),
                    ("prior_beliefs", "/sotoba_node/prior_beliefs"),
                    ("initial_beliefs", "/sotoba_node/initial_beliefs"),
                ],
                condition=IfCondition(predictor),
            ),
            Node(
                package="sotoba_ros",
                executable="fake_scan_publisher",
                name="fake_scan_publisher",
                output="screen",
                condition=IfCondition(fake_scan),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(rviz),
            ),
        ]
    )
