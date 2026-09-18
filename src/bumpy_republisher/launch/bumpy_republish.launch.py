from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_path = get_package_share_directory('bumpy_republisher')

    websocket_host_arg = DeclareLaunchArgument(
        'websocket_host',
        default_value='192.168.1.13',
        description='IP address of the websocket server'
    )

    urdf_file = os.path.join(pkg_path, "urdf/robot.urdf")
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc, "use_sim_time": False}],
    )

    republisher_odom = Node(
        package="bumpy_republisher",
        executable="republish_odom_tf",
        parameters=[{'use_sim_time': False, 'websocket_host': LaunchConfiguration('websocket_host')}]
    )

    republisher_cmd_vel = Node(
        package="bumpy_republisher",
        executable="republish_cmd_vel",
        parameters=[{'use_sim_time': False, 'websocket_host': LaunchConfiguration('websocket_host')}]
    )

    republisher_scan = Node(
        package="bumpy_republisher",
        executable="republish_scan",
        parameters=[{'use_sim_time': False, 'websocket_host': LaunchConfiguration('websocket_host')}]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(pkg_path, "rviz", "odom_test.rviz")],
    )

    return LaunchDescription([
        websocket_host_arg,
        republisher_scan,
        republisher_odom,
        # rviz_node,
        republisher_cmd_vel,
        # robot_state_publisher_node
    ])