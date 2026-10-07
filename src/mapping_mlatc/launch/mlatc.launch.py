from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mapping_mlatc",
            executable="mlatc_node",
            name="mlatc_node",
            output="screen",
        ),
    ])
