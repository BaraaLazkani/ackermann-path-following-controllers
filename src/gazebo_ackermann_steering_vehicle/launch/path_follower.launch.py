#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='pid',
        description='Controller type: pid, stanley, lqr, smc'
    )

    trajectory_arg = DeclareLaunchArgument(
        'trajectory',
        default_value='figure8',
        description='Trajectory type: figure8, lane_change, circle, s_curve'
    )

    # Get package share directory
    pkg_share = FindPackageShare('gazebo_ackermann_steering_vehicle')

    # Determine config file based on controller
    controller_type = LaunchConfiguration('controller')
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        [controller_type, '_params.yaml']
    ])

    # Include vehicle launch file
    vehicle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_share,
                'launch',
                'vehicle.launch.py'
            ])
        ])
    )

    # Odometry to TF broadcaster node (Python)
    tf_broadcaster_node = Node(
        package='gazebo_ackermann_steering_vehicle',
        executable='odometry_to_tf.py',
        name='odometry_to_tf',
        output='screen'
    )

    # Path follower node
    path_follower_node = Node(
        package='gazebo_ackermann_steering_vehicle',
        executable='path_follower_node',
        name='path_follower_node',
        output='screen',
        parameters=[
            config_file,
            {
                'trajectory_type': LaunchConfiguration('trajectory')
            }
        ]
    )

    return LaunchDescription([
        controller_arg,
        trajectory_arg,
        vehicle_launch,
        tf_broadcaster_node,
        path_follower_node
    ])
