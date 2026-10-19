from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _validate_mapping(context):
    selected_mapping = LaunchConfiguration("mapping").perform(context)
    if selected_mapping not in {"atcdt", "mlatc", "both"}:
        raise ValueError(
            "mapping must be one of: atcdt, mlatc, or both "
            f"(got {selected_mapping!r})"
        )
    return []


def generate_launch_description():
    mapping = LaunchConfiguration("mapping")
    atcdt_condition = IfCondition(
        PythonExpression(["'", mapping, "' == 'atcdt' or '", mapping, "' == 'both'"])
    )
    mlatc_condition = IfCondition(
        PythonExpression(["'", mapping, "' == 'mlatc' or '", mapping, "' == 'both'"])
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "mapping",
            default_value="both",
            description="Mapping pipeline to launch: atcdt, mlatc, or both",
        ),
        DeclareLaunchArgument("metrics_csv", default_value="atcdt_metrics.csv"),
        DeclareLaunchArgument("mlatc_metrics_csv", default_value="mlatc_metrics.csv"),
        DeclareLaunchArgument("layer_z_offset", default_value="3.0"),
        DeclareLaunchArgument("show_parent_child", default_value="true"),
        OpaqueFunction(function=_validate_mapping),
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
            output="screen",
            condition=atcdt_condition,
            parameters=[{
                "metrics_csv": LaunchConfiguration("metrics_csv"),
            }],
        ),
        Node(
            package="mapping",
            executable="mlatc_node",
            name="mlatc_node",
            output="screen",
            condition=mlatc_condition,
            parameters=[{
                "layer_z_offset": LaunchConfiguration("layer_z_offset"),
                "show_parent_child": LaunchConfiguration("show_parent_child"),
                "metrics_csv": LaunchConfiguration("mlatc_metrics_csv"),
            }],
        ),
    ])