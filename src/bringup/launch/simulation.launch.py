from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_model = LaunchConfiguration('robot_model')
    world_name = LaunchConfiguration('world')
    record_bag = LaunchConfiguration('record_bag')
    bag_prefix = LaunchConfiguration('bag_prefix')

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value='/home/faza/topoMapWs/src/scenarios/models'
    )

    gazebo = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
            'run',
            PythonExpression([
                "'/home/faza/topoMapWs/src/scenarios/worlds/' + '",
                world_name,
                "' + '.world'"
            ])
        ],
        output='screen'
    )

    spawn_robot = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'run',
                    'ros_gz_sim',
                    'create',
                    '-world',
                    world_name,
                    '-file',
                    PythonExpression([
                        "'/home/faza/topoMapWs/src/scenarios/models/' + '",
                        robot_model,
                        "' + '/model.sdf'"
                    ]),
                    '-name',
                    robot_model,
                    '-x',
                    PythonExpression([
                        "'-3.10' if '",
                        robot_model,
                        "' == 'idealized_lidar' and '",
                        world_name,
                        "' == 'divider_room' else '-8.5'",
                    ]),
                    '-y',
                    PythonExpression([
                        "'-4.0' if '",
                        robot_model,
                        "' == 'idealized_lidar' and '",
                        world_name,
                        "' == 'divider_room' else '8.5'",
                    ]),
                    '-z',
                    PythonExpression([
                        "'0.5' if '",
                        robot_model,
                        "' == 'idealized_lidar' and '",
                        world_name,
                        "' == 'divider_room' else "
                        "'0.65' if '",
                        robot_model,
                        "' == 'idealized_lidar' else '0.16'",
                    ]),
                ],
                output='screen',
            ),
        ],
    )

    remove_divider = Node(
        package='scenarios',
        executable='scenario_event_node',
        parameters=[{
            'world': world_name,
            'model': 'disappearing_wall',
            'remove_after_sec': 30.0,
            'use_sim_time': True,
        }],
        condition=IfCondition(PythonExpression([
            "'",
            world_name,
            "' == 'divider_room'"
        ])),
        output='screen',
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

    preprocess = Node(
        package='preprocess',
        executable='preprocess_node',       # exact name from your CMake add_executable
        parameters=[{
            'robot_name': robot_model,
            'world': world_name,
            'lidar_offset_z': ParameterValue(
                PythonExpression(["0.49 if '", robot_model, "' == 'ddmr' else 0.0"]),
                value_type=float),
            'use_sim_time': True,
        }],
        output='screen',
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen',
    )




    idealized_lidar_controller = Node(
        package='controller',
        executable='idealized_lidar_controller',
        parameters=[{'world': world_name,'use_sim_time': True}],
        condition=IfCondition(PythonExpression([
            "'",
            robot_model,
            "' == 'idealized_lidar'"
        ])),
        output='screen',
    )

    bag_recorder = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'record',
            '-o',
            bag_prefix,
            '-s',
            'sqlite3',
            '/points/points',
            '/mapping/processed_scan',
            '/clock',
            '/tf',
            '/tf_static',
        ],
        condition=IfCondition(record_bag),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_model',
            default_value='ddmr',
            choices=['ddmr', 'idealized_lidar'],
            description='Model to spawn: ddmr or idealized_lidar',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='winding_room',
            choices=['winding_room', 'divider_room'],
            description='Simulation world to launch',
        ),
        DeclareLaunchArgument(
            'record_bag',
            default_value='false',
            description='Whether to record the lidar point cloud and related topics to rosbag2',
        ),
        DeclareLaunchArgument(
            'bag_prefix',
            default_value='topomap_lidar',
            description='Bag filename prefix used by rosbag2 record',
        ),
        gz_resource_path,
        gazebo,
        spawn_robot,
        remove_divider,
        clock_bridge,
        point_bridge,
        cmd_vel_bridge,
        preprocess,
        idealized_lidar_controller,
        bag_recorder,
    ])
