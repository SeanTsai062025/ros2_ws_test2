#!/usr/bin/env python3

# Copyright 2026 Sean Tsai
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch the CSI camera publisher and optional image viewer."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Create the Dexter camera launch description."""
    package_share = get_package_share_directory('dexter_camera')
    config_file = os.path.join(package_share, 'config', 'camera.yaml')

    source = LaunchConfiguration('source')
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    fps = LaunchConfiguration('fps')
    frame_id = LaunchConfiguration('frame_id')
    camera_name = LaunchConfiguration('camera_name')
    camera_info_url = LaunchConfiguration('camera_info_url')
    view = LaunchConfiguration('view')

    return LaunchDescription([
        DeclareLaunchArgument(
            'source',
            default_value='libcamera',
            description='Frame source: libcamera or test',
        ),
        DeclareLaunchArgument('width', default_value='1280'),
        DeclareLaunchArgument('height', default_value='720'),
        DeclareLaunchArgument('fps', default_value='30.0'),
        DeclareLaunchArgument(
            'frame_id', default_value='camera_optical_frame'
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='',
            description='Optional libcamera camera name',
        ),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='',
            description='Optional calibration YAML path or URL',
        ),
        DeclareLaunchArgument(
            'view',
            default_value='true',
            description='Open rqt_image_view',
        ),
        Node(
            package='dexter_camera',
            executable='camera_node',
            namespace='camera',
            name='camera_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'source': ParameterValue(source, value_type=str),
                    'width': ParameterValue(width, value_type=int),
                    'height': ParameterValue(height, value_type=int),
                    'fps': ParameterValue(fps, value_type=float),
                    'frame_id': ParameterValue(frame_id, value_type=str),
                    'camera_name': ParameterValue(
                        camera_name, value_type=str
                    ),
                    'camera_info_url': ParameterValue(
                        camera_info_url, value_type=str
                    ),
                },
            ],
        ),
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='camera_view',
            arguments=['/camera/image_raw'],
            output='screen',
            condition=IfCondition(view),
        ),
    ])
