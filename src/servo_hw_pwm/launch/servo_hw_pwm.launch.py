"""Launch the servo_hw_pwm_node with default parameters."""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('servo_hw_pwm')
    params_file = os.path.join(pkg_share, 'config', 'servo_params.yaml')

    return LaunchDescription([
        Node(
            package='servo_hw_pwm',
            executable='servo_hw_pwm_node',
            name='servo_hw_pwm_node',
            parameters=[params_file],
            output='screen',
            # The node needs /dev/mem access; typically launched via sudo.
            # Alternatively, set appropriate capabilities on the binary.
        ),
    ])
