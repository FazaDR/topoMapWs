from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package="mapping",
            executable="preprocess_node",
            name="preprocess_node",
            output="screen"
        ),

        Node(
            package="mapping",
            executable="topology_node",
            name="topology_node",
            output="screen"
        ),

    ])