import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration("prefix").perform(context)
    config_file = LaunchConfiguration("config_file").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)

    # Build the prefix with underscore.
    # This expression will evaluate to, for example, "cohort_" if
    # the prefix is "cohort", or to an empty string if prefix is empty.
    prefix_ = prefix + "_" if prefix else ""

    # Load config file as a string
    with open(config_file, "r") as f:
        config_str = f.read()

    # Substitute template variables
    config_str = config_str.format(prefix_=prefix_)

    # Load as YAML
    config = yaml.safe_load(config_str)

    # Launch sensor preprocessing nodes
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
                    output="screen",
                    arguments=["--ros-args", "--log-level", log_level],
                    remappings=[
                        ('/tf', 'tf'),
                        ('/tf_static', 'tf_static'),
                        ("cloud_in", node_config["input_cloud_topic"]),
                        ("scan", node_config["output_scan_topic"]),
                    ],
                )
            )

        elif node_type == "pointcloud_transformer":
            nodes.append(
                ComposableNodeContainer(
                    package="rclcpp_components",
                    executable="component_container_mt",
                    name="pointcloud_transformer_container",
                    namespace="",
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
                    output="screen",
                    arguments=["--ros-args", "--log-level", log_level],
                    remappings=[
                        ('/tf', 'tf'),
                        ('/tf_static', 'tf_static'),
                    ],
                )
            )

        elif node_type == "laser_merger2":
            nodes.append(
                Node(
                    package='laser_merger2',
                    executable='laser_merger2',
                    name='laser_merger2',
                    parameters=[{'target_frame': node_config["target_frame"]},
                                {'scan_topics': node_config["scan_topics"]},
                                {'qos_profiles': node_config["qos_profiles"]},
                                {'transform_tolerance': node_config["transform_tolerance"]},
                                {'rate': node_config["rate"]},
                                {'queue_size': node_config["queue_size"]},
                                {'max_range': node_config["max_range"]},
                                {'min_range': node_config["min_range"]},
                                {'max_angle': node_config["max_angle"]},
                                {'min_angle': node_config["min_angle"]},
                                {'scan_time': node_config["scan_time"]},
                                {'angle_increment': node_config["angle_increment"]},
                                {'inf_epsilon': node_config["inf_epsilon"]},
                                {'use_inf': node_config["use_inf"]}
                                ],
                    output='screen',
                    arguments=["--ros-args", "--log-level", log_level],
                    remappings=[
                        ('/tf', 'tf'),
                        ('/tf_static', 'tf_static'),
                        ('pointcloud', node_config["output_pointcloud_topic"]),
                        ('scan', node_config["output_scan_topic"])
                    ],
                ))

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
    default_log_level = "INFO"

    # Declare launch arguments
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace under which to bring up nodes, topics, etc.",
    )
    declare_config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Path to the sensor preprocessor YAML configuration file."
    )
    declare_prefix_arg = DeclareLaunchArgument(
        "prefix",
        default_value="",
        description=(
            "A prefix for the names of joints, links, etc. in the robot model). "
            "E.g. 'base_link' will become 'cohort1_base_link' if prefix "
            "is set to 'cohort1'."
        ),
    )
    declare_log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=default_log_level,
        description="Set the log level for nodes."
    )

    # Launch configurations
    namespace = LaunchConfiguration("namespace")

    # Use PushRosNamespace to apply the namespace to all nodes below
    push_namespace = PushRosNamespace(namespace=namespace)

    return LaunchDescription([
        # Launch arguments
        declare_namespace_arg,
        declare_prefix_arg,
        declare_config_file_arg,
        declare_log_level_arg,
        # declare_use_namespace_arg,
        # Namespace
        push_namespace,
        # Nodes
        OpaqueFunction(function=launch_setup),
    ])
