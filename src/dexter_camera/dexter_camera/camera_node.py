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

"""Publish a Raspberry Pi libcamera stream as ROS 2 Image messages."""

from fractions import Fraction
from pathlib import Path
import sys
from typing import Any
from urllib.parse import unquote
from urllib.parse import urlparse

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import yaml


def _load_gstreamer():
    """Import GStreamer, including from Ubuntu when ROS uses a Conda Python."""
    try:
        import gi
    except ModuleNotFoundError:
        system_packages = Path('/usr/lib/python3/dist-packages')
        if system_packages.is_dir():
            sys.path.append(str(system_packages))
        import gi

    gi.require_version('Gst', '1.0')
    gi.require_version('GstVideo', '1.0')
    from gi.repository import Gst
    from gi.repository import GstVideo

    Gst.init(None)
    return Gst, GstVideo


Gst, GstVideo = _load_gstreamer()


def _gst_quote(value: str) -> str:
    """Quote a string for use as a GStreamer property value."""
    return '"' + value.replace('\\', '\\\\').replace('"', '\\"') + '"'


def build_pipeline(
    source: str,
    width: int,
    height: int,
    fps: float,
    camera_name: str = '',
) -> str:
    """Build the standard libcamera or test GStreamer pipeline."""
    rate = Fraction(str(fps)).limit_denominator(1000)
    caps = (
        f'video/x-raw,width={width},height={height},'
        f'framerate={rate.numerator}/{rate.denominator}'
    )

    if source == 'libcamera':
        camera_property = ''
        if camera_name:
            camera_property = f' camera-name={_gst_quote(camera_name)}'
        source_element = (
            f'libcamerasrc name=source{camera_property} ! {caps}'
        )
    elif source == 'test':
        source_element = (
            f'videotestsrc is-live=true pattern=ball ! {caps}'
        )
    else:
        raise ValueError(
            f'Unsupported source {source!r}; use "libcamera" or "test"'
        )

    return (
        f'{source_element} ! '
        'videoconvert ! video/x-raw,format=RGB ! '
        'queue max-size-buffers=1 leaky=downstream ! '
        'appsink name=sink emit-signals=true max-buffers=1 '
        'drop=true sync=false'
    )


def _resolve_camera_info_url(url: str) -> Path:
    """Resolve a plain, file://, or package:// calibration URL."""
    if url.startswith('package://'):
        relative_url = url[len('package://'):]
        package_name, separator, relative_path = relative_url.partition('/')
        if not separator or not package_name or not relative_path:
            raise ValueError(
                'package:// camera_info_url must include a package and path'
            )
        return (
            Path(get_package_share_directory(package_name)) / relative_path
        )

    if url.startswith('file://'):
        parsed = urlparse(url)
        return Path(unquote(parsed.path))

    return Path(url).expanduser()


def load_camera_info(url: str) -> dict[str, Any] | None:
    """Load and validate a ROS camera_calibration YAML file."""
    if not url:
        return None

    path = _resolve_camera_info_url(url)
    with path.open(encoding='utf-8') as calibration_file:
        data = yaml.safe_load(calibration_file)

    if not isinstance(data, dict):
        raise ValueError(f'Calibration file is not a YAML mapping: {path}')

    matrices = {
        'camera_matrix': 9,
        'rectification_matrix': 9,
        'projection_matrix': 12,
    }
    for key, expected_length in matrices.items():
        values = data.get(key, {}).get('data', [])
        if len(values) != expected_length:
            raise ValueError(
                f'{key}.data in {path} must have {expected_length} values'
            )

    distortion = data.get('distortion_coefficients', {}).get('data', [])
    if not isinstance(distortion, list):
        raise ValueError(
            f'distortion_coefficients.data in {path} must be a list'
        )
    return data


class CameraNode(Node):
    """Capture RGB frames from GStreamer and publish ROS camera messages."""

    def __init__(self) -> None:
        super().__init__('camera_node')

        self.declare_parameter('source', 'libcamera')
        self.declare_parameter('pipeline', '')
        self.declare_parameter('camera_name', '')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('camera_info_topic', 'camera_info')
        self.declare_parameter('camera_info_url', '')
        self.declare_parameter('publish_camera_info', True)

        self._source = self.get_parameter('source').value
        self._width = self.get_parameter('width').value
        self._height = self.get_parameter('height').value
        self._fps = self.get_parameter('fps').value
        self._frame_id = self.get_parameter('frame_id').value
        self._validate_parameters()

        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self._image_publisher = self.create_publisher(
            Image, image_topic, qos_profile_sensor_data
        )
        self._publish_camera_info = self.get_parameter(
            'publish_camera_info'
        ).value
        self._camera_info_publisher = self.create_publisher(
            CameraInfo, camera_info_topic, qos_profile_sensor_data
        )

        calibration_url = self.get_parameter('camera_info_url').value
        try:
            self._calibration = load_camera_info(calibration_url)
        except (OSError, ValueError, yaml.YAMLError) as error:
            raise RuntimeError(
                f'Could not load camera_info_url {calibration_url!r}: {error}'
            ) from error

        custom_pipeline = self.get_parameter('pipeline').value
        if custom_pipeline:
            pipeline_description = custom_pipeline
        else:
            pipeline_description = build_pipeline(
                self._source,
                self._width,
                self._height,
                self._fps,
                self.get_parameter('camera_name').value,
            )

        self._pipeline = None
        self._bus = None
        self._frames = 0
        self._frames_at_last_report = 0
        self._start_pipeline(pipeline_description)
        self._bus_timer = self.create_timer(0.1, self._poll_bus)
        self._statistics_timer = self.create_timer(
            5.0, self._report_statistics
        )

    def _validate_parameters(self) -> None:
        if not isinstance(self._width, int) or self._width <= 0:
            raise ValueError('width must be a positive integer')
        if not isinstance(self._height, int) or self._height <= 0:
            raise ValueError('height must be a positive integer')
        if not isinstance(self._fps, (float, int)) or self._fps <= 0:
            raise ValueError('fps must be positive')
        if self._fps > 240:
            raise ValueError('fps must not exceed 240')
        if not self._frame_id:
            raise ValueError('frame_id must not be empty')

    def _start_pipeline(self, description: str) -> None:
        if self._source == 'libcamera':
            if Gst.ElementFactory.find('libcamerasrc') is None:
                raise RuntimeError(
                    'GStreamer libcamerasrc is unavailable; install '
                    'gstreamer1.0-libcamera'
                )

        try:
            self._pipeline = Gst.parse_launch(description)
        except Gst.ParseError as error:
            raise RuntimeError(
                f'Invalid GStreamer pipeline: {error}'
            ) from error

        sink = self._pipeline.get_by_name('sink')
        if sink is None:
            self._stop_pipeline()
            raise RuntimeError(
                'The GStreamer pipeline must end in an appsink named "sink"'
            )
        sink.connect('new-sample', self._on_new_sample)
        self._bus = self._pipeline.get_bus()

        self.get_logger().info(f'Starting GStreamer pipeline: {description}')
        state_change = self._pipeline.set_state(Gst.State.PLAYING)
        if state_change == Gst.StateChangeReturn.FAILURE:
            details = self._pop_pipeline_error()
            self._stop_pipeline()
            raise RuntimeError(
                'GStreamer could not start the camera pipeline'
                + (f': {details}' if details else '')
            )

        self.get_logger().info(
            f'Publishing {self._width}x{self._height} RGB images at '
            f'{self._fps:g} FPS'
        )

    def _pop_pipeline_error(self) -> str:
        if self._bus is None:
            return ''
        message = self._bus.timed_pop_filtered(
            100 * Gst.MSECOND, Gst.MessageType.ERROR
        )
        if message is None:
            return ''
        error, debug = message.parse_error()
        return f'{error.message} ({debug})' if debug else error.message

    def _on_new_sample(self, sink):
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.ERROR

        caps = sample.get_caps()
        video_info = GstVideo.VideoInfo.new_from_caps(caps)
        if video_info is None:
            self.get_logger().error('Could not read video format from caps')
            return Gst.FlowReturn.ERROR

        buffer = sample.get_buffer()
        mapped, map_info = buffer.map(Gst.MapFlags.READ)
        if not mapped:
            self.get_logger().error('Could not map a camera frame')
            return Gst.FlowReturn.ERROR

        try:
            image = Image()
            image.header.stamp = self.get_clock().now().to_msg()
            image.header.frame_id = self._frame_id
            image.height = video_info.height
            image.width = video_info.width
            image.encoding = 'rgb8'
            image.is_bigendian = 0
            image.step = video_info.stride[0]
            image.data = bytes(map_info.data)
        finally:
            buffer.unmap(map_info)

        self._image_publisher.publish(image)
        if self._publish_camera_info:
            self._camera_info_publisher.publish(
                self._make_camera_info(image)
            )
        self._frames += 1
        return Gst.FlowReturn.OK

    def _make_camera_info(self, image: Image) -> CameraInfo:
        info = CameraInfo()
        info.header = image.header
        info.width = image.width
        info.height = image.height

        if self._calibration is None:
            info.distortion_model = 'plumb_bob'
            info.r = [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0,
            ]
            return info

        info.distortion_model = self._calibration.get(
            'distortion_model', 'plumb_bob'
        )
        info.d = self._calibration[
            'distortion_coefficients'
        ].get('data', [])
        info.k = self._calibration['camera_matrix']['data']
        info.r = self._calibration['rectification_matrix']['data']
        info.p = self._calibration['projection_matrix']['data']
        return info

    def _poll_bus(self) -> None:
        if self._bus is None:
            return

        while True:
            message = self._bus.pop_filtered(
                Gst.MessageType.ERROR
                | Gst.MessageType.EOS
                | Gst.MessageType.WARNING
            )
            if message is None:
                break

            if message.type == Gst.MessageType.WARNING:
                warning, debug = message.parse_warning()
                detail = f' ({debug})' if debug else ''
                self.get_logger().warning(
                    f'GStreamer warning: {warning.message}{detail}'
                )
                continue

            if message.type == Gst.MessageType.ERROR:
                error, debug = message.parse_error()
                detail = f' ({debug})' if debug else ''
                self.get_logger().fatal(
                    f'GStreamer camera error: {error.message}{detail}'
                )
            else:
                self.get_logger().warning(
                    'GStreamer camera stream reached end-of-stream'
                )

            self._stop_pipeline()
            if rclpy.ok():
                rclpy.shutdown()
            break

    def _report_statistics(self) -> None:
        frames = self._frames - self._frames_at_last_report
        self._frames_at_last_report = self._frames
        measured_fps = frames / 5.0
        if self._frames == 0:
            self.get_logger().warning(
                'No camera frames have arrived. Check the CSI connection and '
                'libcamera configuration.'
            )
        else:
            self.get_logger().info(
                f'Camera is publishing at {measured_fps:.1f} FPS'
            )

    def _stop_pipeline(self) -> None:
        if self._pipeline is not None:
            self._pipeline.set_state(Gst.State.NULL)
            self._pipeline = None
            self._bus = None

    def destroy_node(self) -> bool:
        """Stop GStreamer before releasing the ROS node."""
        self._stop_pipeline()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the camera publisher."""
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except (RuntimeError, ValueError) as error:
        if node is None:
            get_logger('camera_startup').fatal(str(error))
        else:
            node.get_logger().fatal(str(error))
        raise SystemExit(1) from error
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                # A second Ctrl+C can arrive while rclpy tears down entities.
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
