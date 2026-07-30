"""Launch the complete arm, Commander, UART bridge, and show orchestrator stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_hardware = LaunchConfiguration('use_hardware')
    use_rviz = LaunchConfiguration('use_rviz')
    show_file = LaunchConfiguration('show_file')
    serial_port = LaunchConfiguration('serial_port')
    safety_confirmed = LaunchConfiguration('safety_confirmed')

    return LaunchDescription([
        DeclareLaunchArgument('use_hardware', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('safety_confirmed', default_value='false'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument(
            'show_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('robot_show_orchestrator'),
                'config',
                'demo_show.yaml',
            ]),
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('dexter_bringup'),
                    'launch',
                    'dexter.launch.xml',
                ])
            ),
            launch_arguments={
                'use_hardware': use_hardware,
                'use_rviz': use_rviz,
            }.items(),
        ),
        Node(
            package='dexter_commander_cpp',
            executable='commander',
            output='screen',
        ),
        Node(
            package='esp32_motor_controller',
            executable='uart_bridge_node',
            parameters=[{'serial_port': serial_port}],
            output='screen',
        ),
        Node(
            package='robot_show_orchestrator',
            executable='show_orchestrator',
            parameters=[{
                'show_file': show_file,
                'auto_start': LaunchConfiguration('auto_start'),
                'safety_confirmed': safety_confirmed,
            }],
            output='screen',
        ),
    ])
