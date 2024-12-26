"""
Copyright 2024 Lorenzo Grandi

GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007

Copyright (C) 2007 Free Software Foundation, Inc. <https://fsf.org/>
Everyone is permitted to copy and distribute verbatim copies
of this license document, but changing it is not allowed.
"""

#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the parent directory
    package_dir = os.path.expanduser('~/ROS2_ws/src/px4_offboard')

    # Define the URDF file paths
    urdf_file_1_path = os.path.join(package_dir, 'urdf', 'iris_depth_camera_1.urdf')
    urdf_file_2_path = os.path.join(package_dir, 'urdf', 'iris_depth_camera_2.urdf')

    # Robot 1 nodes
    robot_state_publisher_1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_1',
        namespace='px4_1',
        output='screen',
        parameters=[{'robot_description': open(urdf_file_1_path).read()}]
    )

    joint_state_publisher_1 = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher_1',
        namespace='px4_1',
        output='screen'
    )

    # Robot 2 nodes
    robot_state_publisher_2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_2',
        namespace='px4_2',
        output='screen',
        parameters=[{'robot_description': open(urdf_file_2_path).read()}]
    )

    joint_state_publisher_2 = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher_2',
        namespace='px4_2',
        output='screen'
    )

    # Other nodes
    processes = Node(
        package='px4_offboard',
        namespace='px4_offboard',
        executable='processes',
        name='processes'
    )

    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        #arguments=['-d', os.path.join(package_dir, 'visualize.rviz')]
        arguments=['-d', os.path.join(package_dir, 'RVIZconfig.rviz')]
    )

    map_publisher = Node(
        package='map_frame_publisher',
        executable='map_frame_publisher',
        name='map_publisher'
    )
    
    map_base_link_publisher = Node(
        package='map_base_link_publ',
        executable='map_base_link_publ',
        name='map_base_link_publisher'
    )

    pointcloud_transformer = Node(
        package='pointcloud_transformer',
        executable='pointcloud_transformer',
        name='pointcloud_transformer'
    )

    pointcloud_combiner = Node(
        package='pointcloud_combiner',
        executable='pointcloud_combiner',
        name='pointcloud_combiner'
    )
    
    # Include other launch files (if needed)
    octomap_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('octomap_server'),
                'launch',
                'octomap_mapping.launch.xml'
            )
        )
    )

    return LaunchDescription([
        robot_state_publisher_1,
        joint_state_publisher_1,
        robot_state_publisher_2,
        joint_state_publisher_2,
        processes,
        rviz,
        map_publisher,
        map_base_link_publisher,
        pointcloud_transformer,
        pointcloud_combiner,
        octomap_launch
    ])
