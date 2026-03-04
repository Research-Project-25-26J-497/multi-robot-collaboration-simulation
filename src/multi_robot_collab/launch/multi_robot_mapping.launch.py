#!/usr/bin/env python3
"""
Multi-Robot Mapping with SLAM Toolbox and RViz Visualization
Launches SLAM for each robot and visualizes the maps in RViz
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('multi_robot_collab')
    
    # Configuration files
    slam_params_file = os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')
    rviz_config_file = os.path.join(pkg_dir, 'config', 'multi_robot_mapping.rviz')
    
    # Robot names
    robot_names = ['robot1', 'robot2', 'robot3', 'robot4']
    
    # Create SLAM nodes for each robot
    slam_nodes = []
    for robot_name in robot_names:
        slam_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=robot_name,
            parameters=[
                slam_params_file,
                {
                    'use_sim_time': True,
                    'odom_frame': f'{robot_name}/odom',
                    'map_frame': 'map',
                    'base_frame': f'{robot_name}/base_link',
                    'scan_topic': f'/{robot_name}/scan',
                }
            ],
            output='screen',
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
        )
        slam_nodes.append(slam_node)
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Map merger node
    map_merger_node = Node(
        package='multi_robot_collab',
        executable='map_merger',
        name='map_merger',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Web bridge node – serves SLAM data to the visualization platform
    web_bridge_node = Node(
        package='multi_robot_collab',
        executable='web_bridge',
        name='web_bridge',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        *slam_nodes,
        rviz_node,
        map_merger_node,
        web_bridge_node,
    ])
