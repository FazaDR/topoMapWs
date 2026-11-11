from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _parse_mapping_selection(value):
    raw_values = [item.strip().lower() for item in str(value).split(",") if item.strip()]
    if not raw_values:
        raise ValueError("mapping selection cannot be empty")

    selected = set()
    for item in raw_values:
        if item == "all":
            selected.update({"atcdt", "atcdt_free", "mlatc", "mlatc_free"})
        elif item == "both":
            selected.update({"atcdt", "mlatc"})
        elif item in {"atcdt", "atcdt_free", "mlatc", "mlatc_free"}:
            selected.add(item)
        else:
            raise ValueError(
                "mapping must be a comma-separated list of: atcdt, atcdt_free, mlatc, mlatc_free, both, all "
                f"(got {value!r})"
            )
    return sorted(selected)


def _build_mapping_nodes(context):
    selected = set(_parse_mapping_selection(LaunchConfiguration("mapping").perform(context)))
    actions = []

    if "atcdt" in selected:
        actions.append(
            Node(
                package="mapping",
                executable="topology_node",
                name="topology_node",
                output="screen",
                parameters=[{
                    "metrics_csv": LaunchConfiguration("metrics_csv"),
                    "sample_period_ms": ParameterValue(
                        LaunchConfiguration("sample_period_ms"),
                        value_type=float,
                    ),
                }],
                remappings=[("/mapping/processed_scan", LaunchConfiguration("input_topic"))],
            )
        )

    if "atcdt_free" in selected:
        actions.append(
            Node(
                package="mapping",
                executable="atcdt_free_node",
                name="atcdt_free_node",
                output="screen",
                parameters=[{
                    "metrics_csv": LaunchConfiguration("atcdt_free_metrics_csv"),
                    "sample_period_ms": ParameterValue(
                        LaunchConfiguration("sample_period_ms"),
                        value_type=float,
                    ),
                }],
                remappings=[("/mapping/processed_scan", LaunchConfiguration("input_topic"))],
            )
        )

    if "mlatc" in selected:
        actions.append(
            Node(
                package="mapping",
                executable="mlatc_node",
                name="mlatc_node",
                output="screen",
                parameters=[{
                    "layer_z_offset": LaunchConfiguration("layer_z_offset"),
                    "show_parent_child": LaunchConfiguration("show_parent_child"),
                    "metrics_csv": LaunchConfiguration("mlatc_metrics_csv"),
                    "sample_period_ms": ParameterValue(
                        LaunchConfiguration("sample_period_ms"),
                        value_type=float,
                    ),
                }],
                remappings=[("/mapping/processed_scan", LaunchConfiguration("input_topic"))],
            )
        )

    if "mlatc_free" in selected:
        actions.append(
            Node(
                package="mapping",
                executable="mlatc_free_node",
                name="mlatc_free_node",
                output="screen",
                parameters=[{
                    "layer_z_offset": LaunchConfiguration("layer_z_offset"),
                    "show_parent_child": LaunchConfiguration("show_parent_child"),
                    "metrics_csv": LaunchConfiguration("mlatc_free_metrics_csv"),
                    "sample_period_ms": ParameterValue(
                        LaunchConfiguration("sample_period_ms"),
                        value_type=float,
                    ),
                }],
                remappings=[("/mapping/processed_scan", LaunchConfiguration("input_topic"))],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mapping",
            default_value="all",
            description="Comma-separated mapping pipelines to launch: atcdt, atcdt_free, mlatc, mlatc_free, both, or all",
        ),
        DeclareLaunchArgument(
            "input_topic",
            default_value="/mapping/processed_scan",
            description="Point cloud topic consumed by the mapping algorithms",
        ),
        DeclareLaunchArgument("metrics_csv", default_value="atcdt_metrics.csv"),
        DeclareLaunchArgument("atcdt_free_metrics_csv", default_value="atcdt_free_metrics.csv"),
        DeclareLaunchArgument("mlatc_metrics_csv", default_value="mlatc_metrics.csv"),
        DeclareLaunchArgument("mlatc_free_metrics_csv", default_value="mlatc_free_metrics.csv"),
        DeclareLaunchArgument("sample_period_ms", default_value="100.0"),
        DeclareLaunchArgument("layer_z_offset", default_value="3.0"),
        DeclareLaunchArgument("show_parent_child", default_value="true"),
        OpaqueFunction(function=_build_mapping_nodes),
    ])