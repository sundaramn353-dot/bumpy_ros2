#!/usr/bin/env python3
"""
bringup.launch.py  –  Bumpy Gamma robot bringup (NO ros2_control)

Nodes launched:
  1. robot_state_publisher   – publishes TF from URDF
  2. diffbot_gpio_node       – pigpio motor driver + encoder → /joint_states
  3. simple_controller       – /joint_states → /odom  +  /cmd_vel → wheel cmds
  4. imu_node                – IMU driver
  5. oled_display_node       – OLED status display
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments ────────────────────────────────────────────────────
    robot_ns_arg = DeclareLaunchArgument(
        "robot_namespace", default_value="bumpy_gamma",
        description="Namespace for the robot (underscores only)")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="False")

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius", default_value="0.022",
        description="Wheel radius in metres (N20 motor, 44 mm diam / 2)")

    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation", default_value="0.0829",
        description="Wheel centre-to-centre distance in metres (calibrated "
                     "2026-09 from 360° rotation test; see pid_params.yaml)")

    enable_tf_arg = DeclareLaunchArgument(
        "enable_tf", default_value="True",
        description="Publish odom→base_footprint TF from simple_controller")

    use_imu_for_angular_z_arg = DeclareLaunchArgument(
        "use_imu_for_angular_z", default_value="False",
        description="Use IMU yaw instead of wheel-odometry for angular heading. "
                    "Only enable when IMU is confirmed publishing a changing yaw.")

    # ── LaunchConfiguration handles ─────────────────────────────────────────
    robot_namespace      = LaunchConfiguration("robot_namespace")
    use_sim_time         = LaunchConfiguration("use_sim_time")
    wheel_radius         = LaunchConfiguration("wheel_radius")
    wheel_separation     = LaunchConfiguration("wheel_separation")
    enable_tf            = LaunchConfiguration("enable_tf")
    use_imu_for_angular_z = LaunchConfiguration("use_imu_for_angular_z")

    # ── Package paths ────────────────────────────────────────────────────────
    pkg_bringup  = get_package_share_directory("diffbot_bringup")
    pkg_gpio     = get_package_share_directory("diffbot_gpio")

    # ── Process URDF (xacro) ─────────────────────────────────────────────────
    urdf_file  = os.path.join(pkg_bringup, "urdf", "diffbot.urdf.xacro")
    robot_desc = xacro.process_file(
        urdf_file,
        mappings={"tf_prefix": "bumpy_gamma"}
    ).toxml()

    # ── GPIO params file ─────────────────────────────────────────────────────
    gpio_params = os.path.join(pkg_gpio, "param", "pid_params.yaml")

    # ── 1. Robot State Publisher ─────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_namespace,
        parameters=[{
            "robot_description": robot_desc,
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )

    # ── 2. GPIO Node  (motor driver + encoder publisher) ─────────────────────
    gpio_node = Node(
        package="diffbot_gpio",
        executable="diffbot_gpio_node",
        name="diffbot_gpio_node",
        namespace=robot_namespace,
        parameters=[gpio_params],
        output="screen",
    )

    # ── 3. Simple Controller (odom + cmd_vel → wheel velocities) ─────────────
    simple_controller = Node(
        package="diffbot_bringup",
        executable="simple_controller",
        name="simple_controller",
        namespace=robot_namespace,
        parameters=[{
            "wheel_radius":          wheel_radius,
            "wheel_separation":      wheel_separation,
            "robot_namespace":       robot_namespace,
            "use_sim_time":          use_sim_time,
            "enable_odom_tf":        enable_tf,
            "use_imu_for_angular_z": use_imu_for_angular_z,
        }],
        output="screen",
    )

    # ── 4. IMU Node (DISABLED — no IMU hardware connected) ───────────────────
    # Uncomment when IMU is physically attached.
    # imu_node = Node(
    #     package="imu_module",
    #     executable="imu_node",
    #     name="imu_node",
    #     namespace=robot_namespace,
    #     output="screen",
    # )

    # ── 5. OLED Display ───────────────────────────────────────────────────────
    oled_node = Node(
        package="diffbot_bringup",
        executable="oled_display",
        name="oled_display_node",
        namespace=robot_namespace,
        output="screen",
    )

    return LaunchDescription([
        robot_ns_arg,
        use_sim_time_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        enable_tf_arg,
        use_imu_for_angular_z_arg,

        robot_state_publisher,
        gpio_node,
        simple_controller,
        # imu_node,   # disabled — no IMU hardware connected
        oled_node,
    ])