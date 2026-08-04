from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value='/home/faza/topoMapWs/src/scenarios/models'
    )

    gazebo = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
            'run',
            '/home/faza/topoMapWs/src/scenarios/worlds/winding_room.world'
        ],
        output='screen'
    )

    point_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )

    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        gazebo,
        point_bridge,
        cmd_vel_bridge,
    ])