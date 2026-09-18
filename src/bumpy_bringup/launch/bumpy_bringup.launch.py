from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Fix: suppress FastDDS XMLPARSER error caused by a stale/missing
    #   FASTRTPS_DEFAULT_PROFILES_FILE environment variable.
    #   Setting it to empty string tells FastDDS to skip loading it.
    fix_fastdds_xml = SetEnvironmentVariable(
        name='FASTRTPS_DEFAULT_PROFILES_FILE',
        value=''
    )

    # ── Launch arguments ──────────────────────────────────────────────────────
    use_imu_for_angular_z_arg = DeclareLaunchArgument(
        "use_imu_for_angular_z",
        default_value="False",
    )

    use_imu_for_angular_z = LaunchConfiguration("use_imu_for_angular_z")

    # ── Rosbridge websocket ───────────────────────────────────────────────────
    pkg_websocket = get_package_share_directory('rosbridge_server')

    websocket_node = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(pkg_websocket, 'launch', 'rosbridge_websocket_launch.xml')
        ),
        launch_arguments={
            'port': '9090',
        }.items()
    )


    # ── LiDAR ────────────────────────────────────────────────────────────────
    # stderr from ydlidar_ros2_driver is driver-level info — not a fatal error.
    lidar_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ydlidar_ros2_driver'),
                'launch',
                'ydlidar_launch.py'
            ])
        ),
    )

    # ── Core diffbot bringup (includes OLED, controllers, RSP, IMU) ──────────
    # stderr from diffbot_gpio is hardware-level info — not a fatal error.
    diffbot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('diffbot_bringup'),
                'launch',
                'bringup.launch.py'
            ])
        ),
        launch_arguments={
            'use_imu_for_angular_z': use_imu_for_angular_z
        }.items()
    )

    # ── Joystick teleop (disabled by default) ─────────────────────────────────
    # joy_teleop = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             FindPackageShare('diffbot_bringup'),
    #             'launch',
    #             'teleop.launch.py'
    #         ])
    #     ),
    # )
    # NOTE: base_footprint -> base_link is already published by
    # robot_state_publisher from the URDF's fixed "base_footprint_joint"
    # (see diffbot_bringup/urdf/diffbot.urdf.xacro). A second, separate
    # static_transform_publisher node used to duplicate this exact edge —
    # removed. robot_state_publisher (started inside diffbot_bringup) is
    # now the single authoritative source for this transform.

    # ── Teleop cmd_vel relay (OPT-IN ONLY) ───────────────────────────────────
    # teleop_twist_keyboard publishes to /cmd_vel (global).
    # simple_controller listens on /bumpy_gamma/cmd_vel (namespaced).
    # This relay bridges the two so keyboard teleop works without remapping.
    #
    # IMPORTANT: this must NEVER run during autonomous Nav2 navigation.
    # collision_monitor is the sole authoritative publisher on
    # /bumpy_gamma/cmd_vel during autonomous operation; a second live
    # publisher on that topic (even if idle) races with it at the DDS layer.
    enable_teleop_relay_arg = DeclareLaunchArgument(
        "enable_teleop_relay",
        default_value="True",
        description="Defaults to True for convenience (teleop-focused setup). "
                     "MUST be overridden to False before autonomous Nav2 "
                     "navigation: "
                     "ros2 launch bumpy_bringup bumpy_bringup.launch.py enable_teleop_relay:=False "
                     "— otherwise this relay will compete with collision_monitor "
                     "on /bumpy_gamma/cmd_vel during autonomous nav.",
    )
    enable_teleop_relay = LaunchConfiguration("enable_teleop_relay")

    teleop_relay = Node(
        package="topic_tools",
        executable="relay",
        name="teleop_cmd_vel_relay",
        arguments=["/cmd_vel", "/bumpy_gamma/cmd_vel"],
        output="screen",
        condition=IfCondition(enable_teleop_relay),
    )

    return LaunchDescription([
        fix_fastdds_xml,          # Must be FIRST — sets env before any node starts
        use_imu_for_angular_z_arg,
        enable_teleop_relay_arg,
        diffbot_bringup,          # Includes OLED display node, RSP
        lidar_publisher,
        # joy_teleop,
        teleop_relay,             # only starts if enable_teleop_relay:=True
       # websocket_node,
    ])
