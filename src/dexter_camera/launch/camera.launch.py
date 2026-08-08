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

"""Launch the camera, gray-object detector, and optional image viewer."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    """Convert a launch argument to a boolean."""
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _launch_setup(
    context,
    config_file,
    detector_config_file,
    system_python_path,
):
    """Create nodes after launch arguments have been resolved."""
    source = LaunchConfiguration('source').perform(context)
    width = int(LaunchConfiguration('width').perform(context))
    height = int(LaunchConfiguration('height').perform(context))
    fps = float(LaunchConfiguration('fps').perform(context))
    pixel_format = LaunchConfiguration('format').perform(context)
    frame_id = LaunchConfiguration('frame_id').perform(context)
    camera_name = LaunchConfiguration('camera_name').perform(context)
    camera_info_url = LaunchConfiguration('camera_info_url').perform(context)
    detect_gray_object = _as_bool(
        LaunchConfiguration('detect_gray_object').perform(context)
    )
    view = _as_bool(LaunchConfiguration('view').perform(context))
    view_topic = LaunchConfiguration('view_topic').perform(context)
    viewer = LaunchConfiguration('viewer').perform(context)
    flip_vertical = _as_bool(
        LaunchConfiguration('flip_vertical').perform(context)
    )
    flip_horizontal = _as_bool(
        LaunchConfiguration('flip_horizontal').perform(context)
    )

    if width <= 0 or height <= 0:
        raise RuntimeError('width and height must be positive')
    if fps <= 0.0 or fps > 240.0:
        raise RuntimeError('fps must be greater than 0 and at most 240')

    nodes = []
    if source == 'libcamera':
        camera_selector = 0
        if camera_name:
            camera_selector = (
                int(camera_name) if camera_name.isdigit() else camera_name
            )

        frame_duration = max(1, round(1_000_000 / fps))
        parameters = {
            'camera': camera_selector,
            'role': 'viewfinder',
            'format': pixel_format,
            'width': width,
            'height': height,
            'frame_id': frame_id,
            'use_node_time': True,
            'FrameDurationLimits': [frame_duration, frame_duration],
        }
        if camera_info_url:
            parameters['camera_info_url'] = camera_info_url

        # camera_ros resolves the workspace's Pi 5-compatible libcamera after
        # install/setup.bash has been sourced.
        nodes.append(Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[parameters],
        ))
    elif source == 'test':
        nodes.append(Node(
            package='dexter_camera',
            executable='camera_node',
            namespace='camera',
            name='camera_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'source': 'test',
                    'width': width,
                    'height': height,
                    'fps': fps,
                    'frame_id': frame_id,
                    'camera_name': camera_name,
                    'camera_info_url': camera_info_url,
                },
            ],
        ))
    else:
        raise RuntimeError(
            f'Unsupported source {source!r}; use "libcamera" or "test"'
        )

    if detect_gray_object:
        nodes.append(Node(
            package='dexter_camera',
            executable='gray_object_detector',
            name='gray_object_detector',
            output='screen',
            parameters=[
                detector_config_file,
                {
                    'flip_vertical': flip_vertical,
                    'flip_horizontal': flip_horizontal,
                },
            ],
        ))

    if view:
        if viewer == 'tuner':
            if not detect_gray_object:
                raise RuntimeError(
                    'viewer="tuner" requires detect_gray_object:=true'
                )
            nodes.append(Node(
                package='dexter_camera',
                executable='color_threshold_tuner',
                name='color_threshold_tuner',
                output='screen',
                parameters=[{
                    'debug_image_topic': view_topic,
                    'raw_image_topic': '/camera/image_raw',
                    'flip_vertical': flip_vertical,
                    'flip_horizontal': flip_horizontal,
                    'sector_topic': '/gray_object/sector',
                    'rgb_threshold_topic': '/gray_object/rgb_thresholds',
                    'area_threshold_topic': '/gray_object/area_thresholds',
                    'candidate_area_topic': '/gray_object/candidate_area',
                    'detector_node': '/gray_object_detector',
                }],
                # The Dexter Conda environment hides Ubuntu's PyQt5 package.
                # Preserve the ROS path while exposing system Qt bindings.
                additional_env={'PYTHONPATH': system_python_path},
            ))
        elif viewer == 'rqt':
            nodes.append(Node(
                package='rqt_image_view',
                executable='rqt_image_view',
                name='camera_view',
                arguments=[view_topic],
                output='screen',
                additional_env={'PYTHONPATH': system_python_path},
            ))
        else:
            raise RuntimeError(
                f'Unsupported viewer {viewer!r}; use "tuner" or "rqt"'
            )

    return nodes


def generate_launch_description():
    """Create the Dexter camera launch description."""
    package_share = get_package_share_directory('dexter_camera')
    config_file = os.path.join(package_share, 'config', 'camera.yaml')
    detector_config_file = os.path.join(
        package_share, 'config', 'gray_object_detector.yaml'
    )
    python_path = os.environ.get('PYTHONPATH', '')
    system_python_path = '/usr/lib/python3/dist-packages'
    if python_path:
        system_python_path += os.pathsep + python_path

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
            'format',
            default_value='RGB888',
            description='libcamera pixel format',
        ),
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
        DeclareLaunchArgument(
            'detect_gray_object',
            default_value='true',
            description='Run the gray-object detector',
        ),
        DeclareLaunchArgument(
            'view_topic',
            default_value='/gray_object/debug_image',
            description='Image topic displayed by the selected viewer',
        ),
        DeclareLaunchArgument(
            'viewer',
            default_value='tuner',
            description='GUI viewer: tuner or rqt',
        ),
        DeclareLaunchArgument(
            'flip_vertical',
            default_value='true',
            description='Swap the top and bottom before detection/display',
        ),
        DeclareLaunchArgument(
            'flip_horizontal',
            default_value='true',
            description='Swap the left and right before detection/display',
        ),
        OpaqueFunction(
            function=_launch_setup,
            kwargs={
                'config_file': config_file,
                'detector_config_file': detector_config_file,
                'system_python_path': system_python_path,
            },
        ),
    ])
