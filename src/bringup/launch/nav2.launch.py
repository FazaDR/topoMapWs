from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare("bringup"),
        "config",
        "nav2_params.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=params_file),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("nav2_bringup"),
                "/launch/navigation_launch.py",
            ]),
            launch_arguments={
                "use_sim_time": "True",
                "autostart": "True",
                "params_file": LaunchConfiguration("params_file"),
            }.items(),
        ),
    ])
