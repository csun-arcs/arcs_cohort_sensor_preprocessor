import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    prefix = LaunchConfiguration("prefix").perform(context)
    config_file = LaunchConfiguration("config_file").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)

    # Log info
    log_info = LogInfo(msg=['Sensor preprocessor bringup launching with namespace: ', namespace, ', prefix: ', prefix])

    # Use PushRosNamespace to apply the namespace to all nodes below
    push_namespace = PushRosNamespace(namespace=namespace)

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
    composable_nodes = []
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
            composable_nodes.append(
                ComposableNode(
                    package="arcs_cohort_sensor_preprocessor",
                    plugin="arcs_cohort_sensor_preprocessor::PointCloudTransformer",
                    name=node_config["name"],
                    parameters=[{
                        "target_frame": node_config["target_frame"],
                        "input_topic": node_config["input_topic"],
                        "output_topic": node_config["output_topic"],
                    }]
                )
            )

        elif node_type == "laser_merger2":

            scan_topics = node_config["scan_topics"] if "scan_topics" in node_config else [""]
            scan_reliability_policies = node_config["scan_reliability_policies"] if "scan_reliability_policies" in node_config else [""]
            scan_history_policies = node_config["scan_history_policies"] if "scan_history_policies" in node_config else [""]
            scan_depths = node_config["scan_depths"] if "scan_depths" in node_config else [20]
            scan_durability_policies = node_config["scan_durability_policies"] if "scan_durability_policies" in node_config else [""]
            point_cloud_topics = node_config["point_cloud_topics"] if "point_cloud_topics" in node_config else [""]
            point_cloud_reliability_policies = node_config["point_cloud_reliability_policies"] if "point_cloud_reliability_policies" in node_config else [""]
            point_cloud_history_policies = node_config["point_cloud_history_policies"] if "point_cloud_history_policies" in node_config else [""]
            point_cloud_depths = node_config["point_cloud_depths"] if "point_cloud_depths" in node_config else [20]
            point_cloud_durability_policies = node_config["point_cloud_durability_policies"] if "point_cloud_durability_policies" in node_config else [""]
            output_scan_reliability_policy = node_config["output_scan_reliability_policy"] if "output_scan_reliability_policy" in node_config else ""
            output_scan_history_policy = node_config["output_scan_history_policy"] if "output_scan_history_policy" in node_config else ""
            output_scan_depth = node_config["output_scan_depth"] if "output_scan_depth" in node_config else 20
            output_scan_durability_policy = node_config["output_scan_durability_policy"] if "output_scan_durability_policy" in node_config else ""
            output_point_cloud_reliability_policy = node_config["output_point_cloud_reliability_policy"] if "output_point_cloud_reliability_policy" in node_config else ""
            output_point_cloud_history_policy = node_config["output_point_cloud_history_policy"] if "output_point_cloud_history_policy" in node_config else ""
            output_point_cloud_depth = node_config["output_point_cloud_depth"] if "output_point_cloud_depth" in node_config else 20
            output_point_cloud_durability_policy = node_config["output_point_cloud_durability_policy"] if "output_point_cloud_durability_policy" in node_config else ""

            nodes.append(
                Node(
                    package="laser_merger2",
                    executable="laser_merger2",
                    name="laser_merger2",
                    parameters=[{"target_frame": node_config["target_frame"]},
                                {"scan_topics": scan_topics},
                                {"scan_reliability_policies": scan_reliability_policies},
                                {"scan_history_policies": scan_history_policies},
                                {"scan_depths": scan_depths},
                                {"scan_durability_policies": scan_durability_policies},
                                {"point_cloud_topics": point_cloud_topics},
                                {"point_cloud_reliability_policies": point_cloud_reliability_policies},
                                {"point_cloud_history_policies": point_cloud_history_policies},
                                {"point_cloud_depths": point_cloud_depths},
                                {"point_cloud_durability_policies": point_cloud_durability_policies},
                                {"output_scan_reliability_policy": output_scan_reliability_policy},
                                {"output_scan_history_policy": output_scan_history_policy},
                                {"output_scan_depth": output_scan_depth},
                                {"output_scan_durability_policy": output_scan_durability_policy},
                                {"output_point_cloud_reliability_policy": output_point_cloud_reliability_policy},
                                {"output_point_cloud_history_policy": output_point_cloud_history_policy},
                                {"output_point_cloud_depth": output_point_cloud_depth},
                                {"output_point_cloud_durability_policy": output_point_cloud_durability_policy},
                                {"transform_tolerance": node_config["transform_tolerance"]},
                                {"rate": node_config["rate"]},
                                {"queue_size": node_config["queue_size"]},
                                {"max_range": node_config["max_range"]},
                                {"min_range": node_config["min_range"]},
                                {"max_angle": node_config["max_angle"]},
                                {"min_angle": node_config["min_angle"]},
                                {"scan_time": node_config["scan_time"]},
                                {"angle_increment": node_config["angle_increment"]},
                                {"inf_epsilon": node_config["inf_epsilon"]},
                                {"use_inf": node_config["use_inf"]}
                                ],
                    output='screen',
                    arguments=["--ros-args", "--log-level", log_level],
                    remappings=[
                        ("/tf", "tf"),
                        ("/tf_static", "tf_static"),
                        ("pointcloud", node_config["output_pointcloud_topic"]),
                        ("scan", node_config["output_scan_topic"])
                    ],
                ))

        elif node_type == "image_transport":

            input_transport = node_config["input_transport"] if "input_transport" in node_config else "raw"
            output_transport = node_config["output_transport"] if "output_transport" in node_config else "raw"
            input_topic = node_config["input_topic"] if "input_topic" in node_config else "in"
            output_topic = node_config["output_topic"] if "output_topic" in node_config else "out"
            jpeg_quality = node_config["jpeg_quality"] if "jpeg_quality" in node_config else 80

            nodes.append(
                Node(
                    package="image_transport",
                    executable="republish",
                    name=node_config["name"],
                    parameters=[
                        {"jpeg_quality": jpeg_quality},
                    ],
                    output="screen",
                    arguments=[input_transport, output_transport, "--ros-args", "--log-level", log_level],
                    remappings=[
                        ("/tf", "tf"),
                        ("/tf_static", "tf_static"),
                        ("in", input_topic),
                        ("out", output_topic),
                        ("out/compressed", [output_topic, "/compressed"]),
                        ("out/compressedDepth", [output_topic, "/compressedDepth"]),
                        ("out/theora", [output_topic, "/theora"]),
                    ],
                )
            )

        elif node_type == "image_resize":

            input_image_topic = node_config["input_image_topic"] if "input_image_topic" in node_config else "image/image_raw"
            input_camera_info_topic = node_config["input_camera_info_topic"] if "input_camera_info_topic" in node_config else "image/camera_info"
            output_image_topic = node_config["output_image_topic"] if "output_image_topic" in node_config else "resize/image_raw"
            output_camera_info_topic = node_config["output_camera_info_topic"] if "output_camera_info_topic" in node_config else "resize/camera_info"
            width = node_config["width"] if "width" in node_config else 320
            height = node_config["height"] if "height" in node_config else 240

            composable_nodes.append(
                ComposableNode(
                    package="image_proc",
                    plugin="image_proc::ResizeNode",
                    name=node_config["name"],
                    remappings=[
                        ("image/image_raw", input_image_topic),
                        ("image/camera_info", input_camera_info_topic),
                        ("resize/image_raw", output_image_topic),
                        ("resize/camera_info", output_camera_info_topic),
                        ("resize/image_raw/compressed", [output_image_topic, "/compressed"]),
                        ("resize/image_raw/compressedDepth", [output_image_topic, "/compressedDepth"]),
                        ("resize/image_raw/theora", [output_image_topic, "/theora"]),
                    ],
                    parameters=[{
                        "width": width,
                        "height": height,
                    }],
                )
            )

        else:
            print(f"[sensor_preprocessor_bringup.launch.py] Warning: Unknown node type '{node_type}'")


    # Create a container for the composable nodes
    nodes.append(
        ComposableNodeContainer(
            package="rclcpp_components",
            executable="component_container",
            name="composable_nodes_container",
            namespace="",
            composable_node_descriptions=composable_nodes,
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
        )
    )

    return [
        GroupAction([
            log_info,
            push_namespace,
            *nodes
        ])
    ]

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

    return LaunchDescription([
        # Launch arguments
        declare_namespace_arg,
        declare_prefix_arg,
        declare_config_file_arg,
        declare_log_level_arg,
        # Nodes
        OpaqueFunction(function=launch_setup),
    ])
