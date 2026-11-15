from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    algorithm = LaunchConfiguration('algorithm')
    free_area = LaunchConfiguration('free_area')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    output_dir = LaunchConfiguration('output_dir')
    replay_bag = LaunchConfiguration('replay_bag')
    replay_rate = LaunchConfiguration('replay_rate')
    replay_storage = LaunchConfiguration('replay_storage')

    default_params = PathJoinSubstitution(
        [FindPackageShare('mapping'), 'config', 'mapping.yaml'])

    mapping_node = Node(
        package='mapping',
        executable='mapping_node',      # must match add_executable in CMakeLists.txt
        name='mapping_node',
        output='screen',
        parameters=[
            params_file,                # base values first, overrides after
            {
                'algorithm': ParameterValue(algorithm, value_type=str),
                'free_area_enabled': ParameterValue(free_area, value_type=bool),
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'metrics.output_dir': ParameterValue(output_dir, value_type=str),
                'replay.bag': ParameterValue(replay_bag, value_type=str),
                'replay.rate': ParameterValue(replay_rate, value_type=float),
                'replay.storage_id': ParameterValue(replay_storage, value_type=str),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'algorithm', default_value='mlatc', choices=['atc', 'mlatc'],
            description='Mapping algorithm'),
        DeclareLaunchArgument(
            'free_area', default_value='true', choices=['true', 'false'],
            description='Enable free-area node deletion'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', choices=['true', 'false'],
            description='Use /clock (Gazebo). Replay mode does not need it.'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='YAML with node parameters'),
        DeclareLaunchArgument(
            'output_dir', default_value='',
            description='Metrics output directory (empty = ~/mapping_metrics)'),
        DeclareLaunchArgument(
            'replay_bag', default_value='',
            description='Bag directory to feed directly into the mapper (empty = live mode)'),
        DeclareLaunchArgument(
            'replay_rate', default_value='0.0',
            description='Replay speed: 0 = as fast as possible, 1.0 = real time'),
        DeclareLaunchArgument(
            'replay_storage', default_value='sqlite3', choices=['sqlite3', 'mcap'],
            description='Bag storage backend'),
        mapping_node,
    ])