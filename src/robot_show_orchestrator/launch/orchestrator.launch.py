"""Launch the UART bridge and show orchestrator (Commander is launched separately)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'show_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('robot_show_orchestrator'),
                'config',
                'demo_show.yaml',
            ]),
        ),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('require_readiness', default_value='true'),
        DeclareLaunchArgument('safety_confirmed', default_value='false'),
        Node(
            package='esp32_motor_controller',
            executable='uart_bridge_node',
            name='uart_bridge_node',
            parameters=[{'serial_port': LaunchConfiguration('serial_port')}],
            output='screen',
        ),
        Node(
            package='robot_show_orchestrator',
            executable='show_orchestrator',
            name='show_orchestrator',
            parameters=[{
                'show_file': LaunchConfiguration('show_file'),
                'auto_start': LaunchConfiguration('auto_start'),
                'require_readiness': LaunchConfiguration('require_readiness'),
                'safety_confirmed': LaunchConfiguration('safety_confirmed'),
            }],
            output='screen',
        ),
    ])
