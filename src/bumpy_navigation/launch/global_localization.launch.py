#!/usr/bin/env python3
"""
global_localization.launch.py  –  Standalone map_server + AMCL
─────────────────────────────────────────────────────────────────────────────
Use this when you want ONLY localization (no full navigation) —
e.g., to verify the robot localizes correctly before adding Nav2.

Map:     bumpy_slam/maps/bumpy_map.yaml
Config:  bumpy_navigation/config/nav2_params.yaml  (amcl section)
─────────────────────────────────────────────────────────────────────────────
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_nav  = get_package_share_directory('bumpy_navigation')
    pkg_slam = get_package_share_directory('bumpy_slam')

    # ── Defaults ─────────────────────────────────────────────────────────────
    default_map    = os.path.join(pkg_slam, 'maps', 'bumpy_map.yaml')
    default_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')

    # ── Launch arguments ─────────────────────────────────────────────────────
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Absolute path to bumpy_map.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Absolute path to nav2_params.yaml')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false')

    map_yaml   = LaunchConfiguration('map')
    params     = LaunchConfiguration('params_file')
    sim_time   = LaunchConfiguration('use_sim_time')

    # ── Map Server ────────────────────────────────────────────────────────────
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params,
            {'yaml_filename': map_yaml,
             'use_sim_time':  sim_time},
        ],
    )

    # ── AMCL ──────────────────────────────────────────────────────────────────
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params, {'use_sim_time': sim_time}],
    )

    # ── Lifecycle Manager ─────────────────────────────────────────────────────
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'node_names':    ['map_server', 'amcl'],
            'use_sim_time':  sim_time,
            'autostart':     True,
        }],
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        sim_time_arg,
        map_server,
        amcl_node,
        lifecycle_manager,
    ])