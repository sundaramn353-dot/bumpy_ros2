#!/usr/bin/env python3
"""
navigation.launch.py  –  Bumpy Gamma full navigation stack
─────────────────────────────────────────────────────────────────────────────
Launches:
  • nav2_bringup  (map_server, amcl, planner, controller, bt_navigator, …)
  • collision_monitor          (independent lifecycle node)
  • velocity_smoother          (wires nav2 cmd_vel → collision_monitor)
  • rviz2                      (pre-configured nav view)

Map:       bumpy_slam/maps/bumpy_map.yaml
Params:    bumpy_navigation/config/nav2_params.yaml
BT:        bumpy_navigation/behavior_trees/navigate_w_replanning_and_recovery.xml

cmd_vel wiring (via SetRemap in GroupAction):
  Nav2 publishes  →  /bumpy_gamma/cmd_vel  (robot's actual topic)
─────────────────────────────────────────────────────────────────────────────
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode, SetRemap


def generate_launch_description():

    # ── Package directories ──────────────────────────────────────────────────
    pkg_nav     = get_package_share_directory('bumpy_navigation')
    pkg_slam    = get_package_share_directory('bumpy_slam')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # ── Paths ────────────────────────────────────────────────────────────────
    default_map = os.path.join(pkg_slam, 'maps', 'bumpy_map.yaml')
    default_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    default_bt  = os.path.join(
        pkg_nav, 'behavior_trees',
        'navigate_w_replanning_and_recovery.xml')
    rviz_config = os.path.join(pkg_nav, 'rviz', 'bumpy_nav_rviz.rviz')

    # ── Launch arguments ─────────────────────────────────────────────────────
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to bumpy_map.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to nav2_params.yaml')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock')

    bt_xml_arg = DeclareLaunchArgument(
        'default_nav_to_pose_bt_xml',
        default_value=default_bt,
        description='BT XML for navigate-to-pose action')

    marker_map_arg = DeclareLaunchArgument(
        'marker_map',
        default_value='{}',
        description='JSON dict of ArUco marker positions: '
                    '\'{"0": [1.0, 2.0, 0.0], "1": [3.0, 0.5, 1.57]}\'')


    # ── LaunchConfigurations ─────────────────────────────────────────────────
    map_yaml   = LaunchConfiguration('map')
    params     = LaunchConfiguration('params_file')
    sim_time   = LaunchConfiguration('use_sim_time')
    bt_xml     = LaunchConfiguration('default_nav_to_pose_bt_xml')
    marker_map = LaunchConfiguration('marker_map')

    # ── Nav2 bringup ─────────────────────────────────────────────────────────
    nav2_group = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'map':          map_yaml,
                    'params_file':  params,
                    'use_sim_time': sim_time,
                }.items(),
            ),
        ]
    )


    # ── RViz2 ────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        sim_time_arg,
        bt_xml_arg,
        marker_map_arg,

        nav2_group,
        rviz_node,
    ])

