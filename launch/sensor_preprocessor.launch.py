from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

import yaml
import os

def load_config():
    config_path = os.path.join(
        get_package_share_directory('arcs_cohort_sensor_preprocessor'),
        'config',
        'sensor_preprocessor.yaml'
    )
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    config = load_config()
    nodes = []

    for node_config in config.get('nodes', []):
        node_type = node_config.get('type')
        if node_type == 'pointcloud_to_laserscan':
            nodes.append(
                Node(
                    package='pointcloud_to_laserscan',
                    executable='pointcloud_to_laserscan_node',
                    name=node_config['name'],
                    parameters=[{
                        'target_frame': node_config['target_frame'],
                        'transform_tolerance': node_config['transform_tolerance'],
                        'min_height': node_config['min_height'],
                        'max_height': node_config['max_height'],
                        'angle_min': node_config['angle_min'],
                        'angle_max': node_config['angle_max'],
                        'angle_increment': node_config['angle_increment'],
                        'range_min': node_config['range_min'],
                        'range_max': node_config['range_max'],
                        'scan_time': node_config['scan_time'],
                        'use_inf': node_config["use_inf"],
                        'inf_epsilon': node_config["inf_epsilon"],
                    }],
                    remappings=[
                        ('cloud_in', node_config['input_cloud_topic']),
                        ('scan', node_config['output_scan_topic'])
                    ],
                    output='screen'
                )
            )
        elif node_type == 'pointcloud_transformer':
            nodes.append(
                ComposableNodeContainer(
                    name='pointcloud_transformer_container',
                    namespace='',
                    package='rclcpp_components',
                    executable='component_container_mt',
                    composable_node_descriptions=[
                        ComposableNode(
                            package='arcs_cohort_sensor_preprocessor',
                            plugin='arcs_cohort_sensor_preprocessor::PointCloudTransformer',
                            name=node_config['name'],
                            parameters=[{
                                'target_frame': node_config['target_frame'],
                                'input_topic': node_config['input_topic'],
                                'output_topic': node_config['output_topic'],
                            }]
                        ),
                    ],
                    output='screen'
                )
            )
        else:
            print("Warning: Unknown node type '{}'".format(node_type))

    return LaunchDescription(nodes)
