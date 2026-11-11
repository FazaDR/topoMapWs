import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _create_metric_directories(context):
    metric_paths = (
        "metrics_csv",
        "atcdt_free_metrics_csv",
        "mlatc_metrics_csv",
        "mlatc_free_metrics_csv",
    )
    for argument_name in metric_paths:
        metric_path = LaunchConfiguration(argument_name).perform(context)
        parent = os.path.dirname(os.path.abspath(metric_path))
        os.makedirs(parent, exist_ok=True)
    return []


def generate_launch_description():
    bag = LaunchConfiguration("bag")
    mapping = LaunchConfiguration("mapping")
    sample_period_ms = LaunchConfiguration("sample_period_ms")
    input_topic = LaunchConfiguration("input_topic")

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("mapping"),
                "launch",
                "mapping.launch.py",
            ])
        ),
        launch_arguments={
            "mapping": mapping,
            "input_topic": input_topic,
            "sample_period_ms": sample_period_ms,
            "metrics_csv": LaunchConfiguration("metrics_csv"),
            "atcdt_free_metrics_csv": LaunchConfiguration("atcdt_free_metrics_csv"),
            "mlatc_metrics_csv": LaunchConfiguration("mlatc_metrics_csv"),
            "mlatc_free_metrics_csv": LaunchConfiguration("mlatc_free_metrics_csv"),
        }.items(),
    )

    bag_player = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag,
            "--clock",
            "--topics",
            "/mapping/processed_scan",
            "--remap",
            ["/mapping/processed_scan:=", input_topic],
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "bag",
            description="Path to a rosbag2 directory containing /points/points",
        ),
        DeclareLaunchArgument(
            "mapping",
            default_value="all",
            description="Mapping pipelines to evaluate: atcdt, atcdt_free, mlatc, mlatc_free, both, or all",
        ),
        DeclareLaunchArgument(
            "input_topic",
            default_value="/mapping/replay_scan",
            description="Topic receiving the replayed processed scan",
        ),
        DeclareLaunchArgument("sample_period_ms", default_value="100.0"),
        DeclareLaunchArgument("metrics_csv", default_value="atcdt_replay_metrics.csv"),
        DeclareLaunchArgument("atcdt_free_metrics_csv", default_value="atcdt_free_replay_metrics.csv"),
        DeclareLaunchArgument("mlatc_metrics_csv", default_value="mlatc_replay_metrics.csv"),
        DeclareLaunchArgument("mlatc_free_metrics_csv", default_value="mlatc_free_replay_metrics.csv"),
        OpaqueFunction(function=_create_metric_directories),
        mapping_launch,
        bag_player,
    ])
