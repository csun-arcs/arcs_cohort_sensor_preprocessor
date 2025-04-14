import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.utilities import perform_substitutions
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("config_file").perform(context)

    with open(config_file, "r") as f:
        config = yaml.safe_load(f)

    nodes = []

    for node_config in config.get("nodes", []):
        node_type = node_config.get("type")

        if node_type == "pointcloud_to_laserscan":
            nodes.append(
                Node(
                    package="pointcloud_to_laserscan",
                    executable="pointcloud_to_laserscan_node",
                    name=node_config["name"],
                    parameters=[{
                        "target_frame": node_config["target_frame"],
                        "transform_tolerance": node_config["transform_tolerance"],
                        "min_height": node_config["min_height"],
                        "max_height": node_config["max_height"],
                        "angle_min": node_config["angle_min"],
                        "angle_max": node_config["angle_max"],
                        "angle_increment": node_config["angle_increment"],
                        "range_min": node_config["range_min"],
                        "range_max": node_config["range_max"],
                        "scan_time": node_config["scan_time"],
                        "use_inf": node_config["use_inf"],
                        "inf_epsilon": node_config["inf_epsilon"],
                    }],
                    remappings=[
                        ("cloud_in", node_config["input_cloud_topic"]),
                        ("scan", node_config["output_scan_topic"]),
                    ],
                    output="screen"
                )
            )

        elif node_type == "pointcloud_transformer":
            nodes.append(
                ComposableNodeContainer(
                    name="pointcloud_transformer_container",
                    namespace="",
                    package="rclcpp_components",
                    executable="component_container_mt",
                    composable_node_descriptions=[
                        ComposableNode(
                            package="arcs_cohort_sensor_preprocessor",
                            plugin="arcs_cohort_sensor_preprocessor::PointCloudTransformer",
                            name=node_config["name"],
                            parameters=[{
                                "target_frame": node_config["target_frame"],
                                "input_topic": node_config["input_topic"],
                                "output_topic": node_config["output_topic"],
                            }]
                        ),
                    ],
                    output="screen"
                )
            )

        else:
            print(f"[sensor_preprocessor_bringup.launch.py] Warning: Unknown node type '{node_type}'")

    return nodes


def generate_launch_description():
    # Packages
    pkg_sensor_prepro = "arcs_cohort_sensor_preprocessor"

    # Defaults
    default_config = os.path.join(
        get_package_share_directory(pkg_sensor_prepro),
        "config",
        "sensor_preprocessor.yaml"
    )

    # Declare launch arguments
    declare_config_file_cmd = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Path to the sensor preprocessor YAML configuration file."
    )

    return LaunchDescription([
        # Launch arguments
        declare_config_file_cmd,
        # Nodes
        OpaqueFunction(function=launch_setup)
    ])
