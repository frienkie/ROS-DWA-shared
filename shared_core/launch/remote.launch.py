#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_share = FindPackageShare('shared_core')
    turtlebot3_bringup_pkg = FindPackageShare('turtlebot3_bringup')
    turtlebot3_description_pkg = FindPackageShare('turtlebot3_description')
    
    # Launch arguments
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='burger',
        description='Robot model: burger or waffle_pi'
    )
    
    # Include turtlebot3 remote launch
    turtlebot3_remote_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                turtlebot3_bringup_pkg,
                'launch',
                'turtlebot3_remote.launch.py'
            ])
        ]),
        launch_arguments={
            'model': LaunchConfiguration('model')
        }.items()
    )
    
    # RViz node
    rviz_config_file = PathJoinSubstitution([
        turtlebot3_description_pkg,
        'rviz',
        LaunchConfiguration('model'),
        'model.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # Filter node
    filter_node = Node(
        package='shared_core',
        executable='filter.py',
        name='filter_node',
        output='screen',
        respawn=False
    )
    
    return LaunchDescription([
        model_arg,
        turtlebot3_remote_launch,
        rviz_node,
        filter_node
    ]) 