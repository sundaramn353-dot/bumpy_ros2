"""
bumpy_teleop.launch.py — Bumpy Gamma, TELEOP mode.

Thin wrapper around bumpy_bringup.launch.py that turns the teleop
cmd_vel relay ON. Use this launch file for manual/keyboard driving.

For autonomous Nav2 navigation, use bumpy_bringup.launch.py directly
(relay stays OFF there) -- never run both modes' relays at the same time
as Nav2's own cmd_vel publisher (collision_monitor).

Usage:
    ros2 launch bumpy_bringup bumpy_teleop.launch.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard   # in another terminal
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bumpy_bringup_dir = get_package_share_directory("bumpy_bringup")

    bringup_in_teleop_mode = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bumpy_bringup_dir, "launch", "bumpy_bringup.launch.py")
        ),
        launch_arguments={"enable_teleop_relay": "True"}.items(),
    )

    return LaunchDescription([
        bringup_in_teleop_mode,
    ])
