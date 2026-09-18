import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch configurations
    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    publish_stamped_twist = launch.substitutions.LaunchConfiguration('publish_stamped_twist')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')
    robot_namespace = launch.substitutions.LaunchConfiguration('robot_namespace')

    return launch.LaunchDescription([

        # ---------------- Launch Arguments ----------------
        # Namespace argument for consistency with bringup.launch.py
        launch.actions.DeclareLaunchArgument(
            'robot_namespace',
            default_value='bumpy_gamma',
            description='Namespace for the robot (use underscores, not hyphens)'
        ),

        launch.actions.DeclareLaunchArgument(
            'joy_vel',
            default_value='/cmd_vel'
        ),

        launch.actions.DeclareLaunchArgument(
            'joy_config',
            default_value='xbox'
        ),

        launch.actions.DeclareLaunchArgument(
            'joy_dev',
            default_value='0'
        ),

        # IMPORTANT: enable TwistStamped
        launch.actions.DeclareLaunchArgument(
            'publish_stamped_twist',
            default_value='true'
        ),

        launch.actions.DeclareLaunchArgument(
            'config_filepath',
            default_value=[
                launch.substitutions.TextSubstitution(
                    text=os.path.join(
                        get_package_share_directory('diffbot_bringup'),
                        'config',
                        ''
                    )
                ),
                joy_config,
                launch.substitutions.TextSubstitution(text='.config.yaml')
            ]
        ),

        # ---------------- Joy Node ----------------
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            namespace=robot_namespace,
            parameters=[{
                'device_id': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }],
        ),

        # ---------------- Teleop Node ----------------
        launch_ros.actions.Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            namespace=robot_namespace,
            parameters=[
                config_filepath,
                {'publish_stamped_twist': publish_stamped_twist}
            ],
            remappings=[
                ('cmd_vel', launch.substitutions.LaunchConfiguration('joy_vel'))
            ],
        ),
    ])
