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

"""Qt live-view and slider controls for gray-object color thresholds."""

import signal
import sys

import numpy as np
from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5 import QtWidgets
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray

from dexter_camera.gray_object_detector import image_message_to_bgr
from dexter_camera.gray_object_detector import orient_bgr_image


THRESHOLD_NAMES = (
    'red_min',
    'red_max',
    'green_min',
    'green_max',
    'blue_min',
    'blue_max',
)
FALLBACK_THRESHOLDS = (55, 225, 55, 225, 55, 225)
AREA_PARAMETER_NAMES = (
    'target_object_area',
    'object_area_tolerance',
)
FALLBACK_AREA_THRESHOLDS = (15850, 1500)


class ImageView(QtWidgets.QLabel):
    """Aspect-ratio image label that reports source-image coordinates."""

    pixel_hovered = QtCore.pyqtSignal(int, int)
    pixel_clicked = QtCore.pyqtSignal(int, int)

    def __init__(self) -> None:
        super().__init__()
        self._image = None
        self._image_rect = QtCore.QRect()
        self._marker = None
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setMinimumSize(480, 270)
        self.setMouseTracking(True)
        self.setStyleSheet('background: #111;')

    def set_bgr_image(self, bgr_image: np.ndarray) -> None:
        """Copy a BGR NumPy image into Qt and display it."""
        rgb_image = np.ascontiguousarray(bgr_image[:, :, ::-1])
        height, width = rgb_image.shape[:2]
        bytes_per_line = width * 3
        self._image = QtGui.QImage(
            rgb_image.data,
            width,
            height,
            bytes_per_line,
            QtGui.QImage.Format_RGB888,
        ).copy()
        self._render_image()

    def _render_image(self) -> None:
        if self._image is None:
            return
        pixmap = QtGui.QPixmap.fromImage(self._image).scaled(
            self.size(),
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation,
        )
        left = (self.width() - pixmap.width()) // 2
        top = (self.height() - pixmap.height()) // 2
        self._image_rect = QtCore.QRect(
            left,
            top,
            pixmap.width(),
            pixmap.height(),
        )
        if self._marker is not None:
            marker_x, marker_y = self._marker
            display_x = round(
                marker_x * pixmap.width() / self._image.width()
            )
            display_y = round(
                marker_y * pixmap.height() / self._image.height()
            )
            painter = QtGui.QPainter(pixmap)
            painter.setRenderHint(QtGui.QPainter.Antialiasing)
            painter.setPen(QtGui.QPen(QtGui.QColor(255, 220, 0), 3))
            painter.drawEllipse(
                QtCore.QPoint(display_x, display_y), 9, 9
            )
            painter.drawLine(
                display_x - 14, display_y, display_x + 14, display_y
            )
            painter.drawLine(
                display_x, display_y - 14, display_x, display_y + 14
            )
            painter.end()
        self.setPixmap(pixmap)

    def set_marker(self, x: int | None, y: int | None) -> None:
        """Set or clear the locked source-pixel marker."""
        self._marker = None if x is None or y is None else (x, y)
        self._render_image()

    def _source_coordinates(self, position) -> tuple[int, int] | None:
        if (
            self._image is None
            or not self._image_rect.contains(position)
        ):
            return None
        relative_x = position.x() - self._image_rect.left()
        relative_y = position.y() - self._image_rect.top()
        source_x = min(
            self._image.width() - 1,
            relative_x * self._image.width()
            // max(1, self._image_rect.width()),
        )
        source_y = min(
            self._image.height() - 1,
            relative_y * self._image.height()
            // max(1, self._image_rect.height()),
        )
        return source_x, source_y

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._render_image()

    def mouseMoveEvent(self, event) -> None:
        coordinates = self._source_coordinates(event.pos())
        if coordinates is None:
            self.pixel_hovered.emit(-1, -1)
            return
        self.pixel_hovered.emit(*coordinates)

    def mousePressEvent(self, event) -> None:
        super().mousePressEvent(event)
        if event.button() != QtCore.Qt.LeftButton:
            return
        coordinates = self._source_coordinates(event.pos())
        if coordinates is not None:
            self.pixel_clicked.emit(*coordinates)

    def leaveEvent(self, event) -> None:
        super().leaveEvent(event)
        self.pixel_hovered.emit(-1, -1)


class ThresholdRow(QtWidgets.QWidget):
    """A named slider paired with an exact numeric spin box."""

    value_changed = QtCore.pyqtSignal(int)

    def __init__(self, title: str, value: int) -> None:
        super().__init__()
        title_label = QtWidgets.QLabel(title)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(0, 255)
        self.slider.setValue(value)
        self.spin_box = QtWidgets.QSpinBox()
        self.spin_box.setRange(0, 255)
        self.spin_box.setValue(value)
        self.spin_box.setFixedWidth(70)

        self.slider.valueChanged.connect(self.spin_box.setValue)
        self.spin_box.valueChanged.connect(self.slider.setValue)
        self.slider.valueChanged.connect(self.value_changed.emit)

        layout = QtWidgets.QGridLayout(self)
        layout.setContentsMargins(0, 2, 0, 8)
        layout.addWidget(title_label, 0, 0, 1, 2)
        layout.addWidget(self.slider, 1, 0)
        layout.addWidget(self.spin_box, 1, 1)

    def value(self) -> int:
        return self.slider.value()

    def set_value(self, value: int) -> None:
        self.slider.setValue(value)

    def set_range(self, minimum: int, maximum: int) -> None:
        self.slider.setRange(minimum, maximum)
        self.spin_box.setRange(minimum, maximum)


class ColorTunerWindow(QtWidgets.QMainWindow):
    """Show detector output and live RGB threshold controls side by side."""

    def __init__(self, controller) -> None:
        super().__init__()
        self._controller = controller
        self._raw_image = None
        self._hovered_pixel = (-1, -1)
        self._locked_pixel = None
        self._candidate_area = 0
        self._startup_thresholds = None
        self._startup_area_thresholds = None
        self.setWindowTitle('Dexter RGB Color Threshold Tuner')
        self.resize(1180, 720)

        self.image_view = ImageView()
        self.image_view.pixel_hovered.connect(self._show_pixel)
        self.image_view.pixel_clicked.connect(self._lock_pixel)

        controls = QtWidgets.QWidget()
        controls.setMinimumWidth(340)
        controls.setMaximumWidth(440)
        control_layout = QtWidgets.QVBoxLayout(controls)

        title = QtWidgets.QLabel('RGB Color Range')
        title.setStyleSheet('font-size: 18px; font-weight: bold;')
        control_layout.addWidget(title)

        explanation = QtWidgets.QLabel(
            'Adjust the minimum and maximum values for R, G, and B. '
            'Pixels inside the selected range are highlighted in green '
            'immediately. Click the image to lock a pixel.'
        )
        explanation.setWordWrap(True)
        control_layout.addWidget(explanation)

        self._rows = {
            'red_min': ThresholdRow(
                'Red minimum',
                FALLBACK_THRESHOLDS[0],
            ),
            'red_max': ThresholdRow(
                'Red maximum',
                FALLBACK_THRESHOLDS[1],
            ),
            'green_min': ThresholdRow(
                'Green minimum',
                FALLBACK_THRESHOLDS[2],
            ),
            'green_max': ThresholdRow(
                'Green maximum',
                FALLBACK_THRESHOLDS[3],
            ),
            'blue_min': ThresholdRow(
                'Blue minimum',
                FALLBACK_THRESHOLDS[4],
            ),
            'blue_max': ThresholdRow(
                'Blue maximum',
                FALLBACK_THRESHOLDS[5],
            ),
        }
        for row in self._rows.values():
            row.setEnabled(False)
            row.value_changed.connect(self._threshold_changed)
            control_layout.addWidget(row)

        pixel_group = QtWidgets.QGroupBox('Pixel RGB at Cursor')
        pixel_layout = QtWidgets.QHBoxLayout(pixel_group)
        self._color_swatch = QtWidgets.QFrame()
        self._color_swatch.setFixedSize(64, 64)
        self._color_swatch.setStyleSheet(
            'background: #222; border: 1px solid #777;'
        )
        self._pixel_text = QtWidgets.QLabel(
            'Move the pointer over the image'
        )
        self._pixel_text.setMinimumWidth(200)
        pixel_layout.addWidget(self._color_swatch)
        pixel_layout.addWidget(self._pixel_text, 1)
        control_layout.addWidget(pixel_group)

        selected_layout = QtWidgets.QGridLayout()
        selected_layout.addWidget(
            QtWidgets.QLabel('Color tolerance'), 0, 0
        )
        self._tolerance_box = QtWidgets.QSpinBox()
        self._tolerance_box.setRange(0, 127)
        self._tolerance_box.setValue(20)
        selected_layout.addWidget(self._tolerance_box, 0, 1)
        apply_color_button = QtWidgets.QPushButton(
            'Apply locked color +/- tolerance'
        )
        apply_color_button.clicked.connect(self._apply_locked_color)
        clear_lock_button = QtWidgets.QPushButton('Unlock pixel')
        clear_lock_button.clicked.connect(self._clear_lock)
        selected_layout.addWidget(apply_color_button, 1, 0)
        selected_layout.addWidget(clear_lock_button, 1, 1)
        control_layout.addLayout(selected_layout)

        self._area_group = QtWidgets.QGroupBox(
            'Little Guy Area Lock'
        )
        area_layout = QtWidgets.QGridLayout(self._area_group)
        self._current_area_label = QtWidgets.QLabel(
            'Current candidate area: 0 px'
        )
        area_layout.addWidget(self._current_area_label, 0, 0, 1, 2)
        area_layout.addWidget(QtWidgets.QLabel('Target area'), 1, 0)
        self._target_area_box = QtWidgets.QSpinBox()
        self._target_area_box.setRange(0, 10_000_000)
        self._target_area_box.setValue(FALLBACK_AREA_THRESHOLDS[0])
        area_layout.addWidget(self._target_area_box, 1, 1)
        area_layout.addWidget(QtWidgets.QLabel('Area tolerance +/-'), 2, 0)
        self._area_tolerance_box = QtWidgets.QSpinBox()
        self._area_tolerance_box.setRange(0, 10_000_000)
        self._area_tolerance_box.setValue(FALLBACK_AREA_THRESHOLDS[1])
        area_layout.addWidget(self._area_tolerance_box, 2, 1)
        self._area_range_label = QtWidgets.QLabel()
        area_layout.addWidget(self._area_range_label, 3, 0, 1, 2)
        use_current_area_button = QtWidgets.QPushButton(
            'Use current area as target'
        )
        use_current_area_button.clicked.connect(self._use_current_area)
        area_layout.addWidget(use_current_area_button, 4, 0, 1, 2)
        self._target_area_box.valueChanged.connect(
            self._area_threshold_changed
        )
        self._area_tolerance_box.valueChanged.connect(
            self._area_threshold_changed
        )
        self._area_group.setEnabled(False)
        control_layout.addWidget(self._area_group)

        self._sector_label = QtWidgets.QLabel('Little Guy Sector: --')
        self._sector_label.setStyleSheet(
            'font-size: 16px; font-weight: bold;'
        )
        control_layout.addWidget(self._sector_label)

        button_layout = QtWidgets.QHBoxLayout()
        reload_button = QtWidgets.QPushButton('Reload settings')
        reload_button.clicked.connect(controller.reload_parameters)
        reset_button = QtWidgets.QPushButton('Reset to startup')
        reset_button.clicked.connect(self._reset_thresholds)
        button_layout.addWidget(reload_button)
        button_layout.addWidget(reset_button)
        control_layout.addLayout(button_layout)
        control_layout.addStretch(1)

        central = QtWidgets.QWidget()
        central_layout = QtWidgets.QHBoxLayout(central)
        central_layout.setContentsMargins(8, 8, 8, 8)
        central_layout.addWidget(self.image_view, 1)
        control_scroll = QtWidgets.QScrollArea()
        control_scroll.setWidgetResizable(True)
        control_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        control_scroll.setMinimumWidth(360)
        control_scroll.setMaximumWidth(460)
        control_scroll.setWidget(controls)
        central_layout.addWidget(control_scroll)
        self.setCentralWidget(central)
        self.statusBar().showMessage('Waiting for gray_object_detector...')
        self._sync_slider_ranges()

    def set_debug_image(self, image: np.ndarray) -> None:
        self.image_view.set_bgr_image(image)

    def set_raw_image(self, image: np.ndarray) -> None:
        self._raw_image = image
        self._refresh_hovered_pixel()

    def set_sector(self, sector: int) -> None:
        self._sector_label.setText(f'Little Guy Sector: {sector}')

    def set_connection_status(self, message: str) -> None:
        self.statusBar().showMessage(message)

    def set_controls_enabled(self, enabled: bool) -> None:
        """Enable sliders only after the detector values are loaded."""
        for row in self._rows.values():
            row.setEnabled(enabled)
        self._area_group.setEnabled(enabled)

    def thresholds(self) -> tuple[int, int, int, int, int, int]:
        return tuple(
            self._rows[name].value() for name in THRESHOLD_NAMES
        )

    def set_thresholds(self, values) -> None:
        blockers = [
            QtCore.QSignalBlocker(self._rows[name])
            for name in THRESHOLD_NAMES
        ]
        slider_blockers = [
            QtCore.QSignalBlocker(self._rows[name].slider)
            for name in THRESHOLD_NAMES
        ]
        spin_blockers = [
            QtCore.QSignalBlocker(self._rows[name].spin_box)
            for name in THRESHOLD_NAMES
        ]
        for name, value in zip(THRESHOLD_NAMES, values):
            self._rows[name].set_range(0, 255)
            self._rows[name].set_value(int(value))
            self._rows[name].spin_box.setValue(int(value))
        del blockers, slider_blockers, spin_blockers
        self._sync_slider_ranges()
        for row in self._rows.values():
            row.setEnabled(True)
        if self._startup_thresholds is None:
            self._startup_thresholds = tuple(int(value) for value in values)
        self._refresh_hovered_pixel()

    def set_area_thresholds(self, target: int, tolerance: int) -> None:
        """Load area lock values without sending them back to ROS."""
        target_blocker = QtCore.QSignalBlocker(self._target_area_box)
        tolerance_blocker = QtCore.QSignalBlocker(
            self._area_tolerance_box
        )
        self._target_area_box.setValue(int(target))
        self._area_tolerance_box.setValue(int(tolerance))
        del target_blocker, tolerance_blocker
        if self._startup_area_thresholds is None:
            self._startup_area_thresholds = (int(target), int(tolerance))
        self._refresh_area_status()

    def set_candidate_area(self, area: int) -> None:
        """Show the largest color-connected region's current pixel area."""
        self._candidate_area = int(area)
        self._refresh_area_status()

    def _area_threshold_changed(self, _value: int) -> None:
        target = self._target_area_box.value()
        tolerance = self._area_tolerance_box.value()
        self._refresh_area_status()
        self._controller.request_area_thresholds(target, tolerance)

    def _use_current_area(self) -> None:
        if self._candidate_area <= 0:
            self.set_connection_status(
                'No color-region area is currently available'
            )
            return
        self._target_area_box.setValue(self._candidate_area)

    def _refresh_area_status(self) -> None:
        target = self._target_area_box.value()
        tolerance = self._area_tolerance_box.value()
        minimum = max(0, target - tolerance)
        maximum = target + tolerance
        area_matches = (
            self._candidate_area > 0
            and minimum <= self._candidate_area <= maximum
        )
        status = 'AREA MATCH: LOCKED' if area_matches else 'NO AREA MATCH'
        color = '#35c759' if area_matches else '#ff453a'
        self._current_area_label.setText(
            f'Current candidate area: {self._candidate_area} px'
        )
        self._area_range_label.setText(
            f'Allowed range: {minimum}..{maximum} px  '
            f'<b style="color:{color}">{status}</b>'
        )

    def _sync_slider_ranges(self) -> None:
        values = self.thresholds()
        for index, channel in enumerate(('red', 'green', 'blue')):
            minimum = values[index * 2]
            maximum = values[index * 2 + 1]
            self._rows[f'{channel}_min'].set_range(0, maximum)
            self._rows[f'{channel}_max'].set_range(minimum, 255)

    def _threshold_changed(self, _value: int) -> None:
        self._sync_slider_ranges()
        self._refresh_hovered_pixel()
        self._controller.request_thresholds(self.thresholds())

    def _reset_thresholds(self) -> None:
        values = self._startup_thresholds or FALLBACK_THRESHOLDS
        self.set_thresholds(values)
        self._controller.request_thresholds(self.thresholds())
        area_values = (
            self._startup_area_thresholds or FALLBACK_AREA_THRESHOLDS
        )
        self.set_area_thresholds(*area_values)
        self._controller.request_area_thresholds(*area_values)

    def _show_pixel(self, x: int, y: int) -> None:
        self._hovered_pixel = (x, y)
        if self._locked_pixel is None:
            self._refresh_hovered_pixel()

    def _lock_pixel(self, x: int, y: int) -> None:
        self._locked_pixel = (x, y)
        self.image_view.set_marker(x, y)
        self._refresh_hovered_pixel()

    def _clear_lock(self) -> None:
        self._locked_pixel = None
        self._hovered_pixel = (-1, -1)
        self.image_view.set_marker(None, None)
        self._refresh_hovered_pixel()

    def _sampled_rgb(self):
        coordinates = self._locked_pixel or self._hovered_pixel
        x, y = coordinates
        if (
            self._raw_image is None
            or x < 0
            or y < 0
            or y >= self._raw_image.shape[0]
            or x >= self._raw_image.shape[1]
        ):
            return None
        blue, green, red = (
            int(value) for value in self._raw_image[y, x]
        )
        return x, y, red, green, blue

    def _apply_locked_color(self) -> None:
        if self._locked_pixel is None:
            self.set_connection_status(
                'Click the image to lock a color first'
            )
            return
        sampled = self._sampled_rgb()
        if sampled is None:
            return
        _, _, red, green, blue = sampled
        tolerance = self._tolerance_box.value()
        values = tuple(
            value
            for channel_value in (red, green, blue)
            for value in (
                max(0, channel_value - tolerance),
                min(255, channel_value + tolerance),
            )
        )
        self.set_thresholds(values)
        self._controller.request_thresholds(values)

    def _refresh_hovered_pixel(self) -> None:
        sampled = self._sampled_rgb()
        if sampled is None:
            self._pixel_text.setText(
                'Move the pointer to inspect; click to lock a pixel'
            )
            self._color_swatch.setStyleSheet(
                'background: #222; border: 1px solid #777;'
            )
            return

        x, y, red, green, blue = sampled
        red_min, red_max, green_min, green_max, blue_min, blue_max = (
            self.thresholds()
        )
        accepted = (
            red_min <= red <= red_max
            and green_min <= green <= green_max
            and blue_min <= blue <= blue_max
        )
        result_text = 'MATCH' if accepted else 'NO MATCH'
        result_color = '#35c759' if accepted else '#ff453a'
        sample_state = (
            'LOCKED' if self._locked_pixel is not None else 'PREVIEW'
        )
        self._pixel_text.setText(
            f'{sample_state} ({x}, {y})\n'
            f'R {red}   G {green}   B {blue}\n'
            f'<b style="color:{result_color}">{result_text}</b>'
        )
        self._color_swatch.setStyleSheet(
            f'background: rgb({red}, {green}, {blue}); '
            'border: 1px solid #777;'
        )


class ColorTunerNode(Node):
    """Bridge ROS images and detector parameters into the Qt window."""

    def __init__(self) -> None:
        super().__init__('color_threshold_tuner')
        self.declare_parameter(
            'debug_image_topic', '/gray_object/debug_image'
        )
        self.declare_parameter('raw_image_topic', '/camera/image_raw')
        self.declare_parameter('flip_vertical', True)
        self.declare_parameter('flip_horizontal', True)
        self.declare_parameter('sector_topic', '/gray_object/sector')
        self.declare_parameter(
            'rgb_threshold_topic', '/gray_object/rgb_thresholds'
        )
        self.declare_parameter(
            'area_threshold_topic', '/gray_object/area_thresholds'
        )
        self.declare_parameter(
            'candidate_area_topic', '/gray_object/candidate_area'
        )
        self.declare_parameter(
            'detector_node', '/gray_object_detector'
        )

        debug_topic = self.get_parameter('debug_image_topic').value
        raw_topic = self.get_parameter('raw_image_topic').value
        self._flip_vertical = self.get_parameter(
            'flip_vertical'
        ).value
        self._flip_horizontal = self.get_parameter(
            'flip_horizontal'
        ).value
        if not isinstance(self._flip_vertical, bool):
            raise ValueError('flip_vertical must be a boolean')
        if not isinstance(self._flip_horizontal, bool):
            raise ValueError('flip_horizontal must be a boolean')
        sector_topic = self.get_parameter('sector_topic').value
        rgb_threshold_topic = self.get_parameter(
            'rgb_threshold_topic'
        ).value
        area_threshold_topic = self.get_parameter(
            'area_threshold_topic'
        ).value
        candidate_area_topic = self.get_parameter(
            'candidate_area_topic'
        ).value
        detector_node = self.get_parameter('detector_node').value

        self._window = None
        self._reported_image_error = False
        self._connected = False
        self._get_future = None
        self._set_future = None
        self._pending_thresholds = None
        self._area_set_future = None
        self._pending_area_thresholds = None
        self._parameter_sync_available = True
        self._parameter_client = AsyncParameterClient(
            self, detector_node
        )
        self._debug_subscription = self.create_subscription(
            Image,
            debug_topic,
            self._on_debug_image,
            qos_profile_sensor_data,
        )
        self._raw_subscription = self.create_subscription(
            Image,
            raw_topic,
            self._on_raw_image,
            qos_profile_sensor_data,
        )
        self._sector_subscription = self.create_subscription(
            Int32,
            sector_topic,
            self._on_sector,
            10,
        )
        self._threshold_publisher = self.create_publisher(
            Int32MultiArray,
            rgb_threshold_topic,
            10,
        )
        self._area_threshold_publisher = self.create_publisher(
            Int32MultiArray,
            area_threshold_topic,
            10,
        )
        self._candidate_area_subscription = self.create_subscription(
            Int32,
            candidate_area_topic,
            self._on_candidate_area,
            10,
        )

    def attach_window(self, window: ColorTunerWindow) -> None:
        self._window = window

    def poll_remote(self) -> None:
        """Connect and load the detector's current threshold parameters."""
        if self._connected or self._get_future is not None:
            return
        if not self._parameter_client.services_are_ready():
            return
        self._get_future = self._parameter_client.get_parameters(
            list(THRESHOLD_NAMES + AREA_PARAMETER_NAMES),
            callback=self._parameters_received,
        )

    def reload_parameters(self) -> None:
        self._connected = False
        self._get_future = None
        self._pending_thresholds = None
        self._pending_area_thresholds = None
        self._parameter_sync_available = True
        if self._window is not None:
            self._window.set_controls_enabled(False)
            self._window.set_connection_status(
                'Reloading detector thresholds...'
            )

    def request_thresholds(self, values) -> None:
        """Publish immediately and also synchronize detector parameters."""
        values = tuple(int(value) for value in values)
        message = Int32MultiArray()
        message.data = list(values)
        self._threshold_publisher.publish(message)
        self._pending_thresholds = values
        self._send_pending_thresholds()

    def request_area_thresholds(
        self,
        target: int,
        tolerance: int,
    ) -> None:
        """Publish target pixel area and tolerance immediately."""
        values = (int(target), int(tolerance))
        message = Int32MultiArray()
        message.data = list(values)
        self._area_threshold_publisher.publish(message)
        self._pending_area_thresholds = values
        self._send_pending_area_thresholds()

    def _parameters_received(self, future) -> None:
        self._get_future = None
        try:
            response = future.result()
            values = tuple(
                Parameter.from_parameter_msg(value).value
                for value in response.values
            )
            expected_count = len(THRESHOLD_NAMES + AREA_PARAMETER_NAMES)
            if len(values) != expected_count:
                raise RuntimeError('detector returned incomplete thresholds')
        except Exception as error:  # ROS futures expose several error types.
            if self._window is not None:
                self._window.set_connection_status(
                    f'Could not read detector thresholds: {error}'
                )
            return

        self._connected = True
        if self._window is not None:
            self._window.set_thresholds(values[:len(THRESHOLD_NAMES)])
            self._window.set_area_thresholds(
                values[-2],
                values[-1],
            )
            self._window.set_controls_enabled(True)
            self._window.set_connection_status(
                'Connected: sliders update gray_object_detector live'
            )
        self._send_pending_thresholds()
        self._send_pending_area_thresholds()

    def _send_pending_thresholds(self) -> None:
        if (
            not self._connected
            or not self._parameter_sync_available
            or self._set_future is not None
            or self._pending_thresholds is None
        ):
            return
        values = self._pending_thresholds
        self._pending_thresholds = None
        parameters = [
            Parameter(name, value=value)
            for name, value in zip(THRESHOLD_NAMES, values)
        ]
        self._set_future = (
            self._parameter_client.set_parameters_atomically(
                parameters,
                callback=self._thresholds_sent,
            )
        )

    def _thresholds_sent(self, future) -> None:
        self._set_future = None
        try:
            result = future.result().result
            if not result.successful:
                raise RuntimeError(result.reason or 'update was rejected')
        except Exception as error:  # ROS futures expose several error types.
            self._parameter_sync_available = False
            self._pending_thresholds = None
            if self._window is not None:
                self._window.set_connection_status(
                    'Live RGB updates remain active; parameter sync failed: '
                    f'{error}'
                )
            return

        if self._window is not None:
            self._window.set_connection_status(
                'Connected: detector updated live'
            )
        self._send_pending_thresholds()

    def _send_pending_area_thresholds(self) -> None:
        if (
            not self._connected
            or not self._parameter_sync_available
            or self._area_set_future is not None
            or self._pending_area_thresholds is None
        ):
            return
        values = self._pending_area_thresholds
        self._pending_area_thresholds = None
        parameters = [
            Parameter(name, value=value)
            for name, value in zip(AREA_PARAMETER_NAMES, values)
        ]
        self._area_set_future = (
            self._parameter_client.set_parameters_atomically(
                parameters,
                callback=self._area_thresholds_sent,
            )
        )

    def _area_thresholds_sent(self, future) -> None:
        self._area_set_future = None
        try:
            result = future.result().result
            if not result.successful:
                raise RuntimeError(result.reason or 'update was rejected')
        except Exception as error:  # ROS futures expose several error types.
            self._parameter_sync_available = False
            self._pending_area_thresholds = None
            if self._window is not None:
                self._window.set_connection_status(
                    'Live area updates remain active; parameter sync failed: '
                    f'{error}'
                )
            return
        self._send_pending_area_thresholds()

    def _convert_image(self, message: Image):
        try:
            return image_message_to_bgr(message)
        except ValueError as error:
            if not self._reported_image_error:
                self.get_logger().error(str(error))
                self._reported_image_error = True
            return None

    def _on_debug_image(self, message: Image) -> None:
        image = self._convert_image(message)
        if image is not None and self._window is not None:
            self._window.set_debug_image(image)

    def _on_raw_image(self, message: Image) -> None:
        image = self._convert_image(message)
        if image is not None and self._window is not None:
            image = orient_bgr_image(
                image,
                self._flip_vertical,
                self._flip_horizontal,
            )
            self._window.set_raw_image(image)

    def _on_sector(self, message: Int32) -> None:
        if self._window is not None:
            self._window.set_sector(message.data)

    def _on_candidate_area(self, message: Int32) -> None:
        if self._window is not None:
            self._window.set_candidate_area(message.data)


def main(args=None) -> None:
    """Run Qt and ROS callbacks cooperatively in the GUI thread."""
    rclpy.init(args=args)
    application = QtWidgets.QApplication(sys.argv)
    node = ColorTunerNode()
    window = ColorTunerWindow(node)
    node.attach_window(window)
    window.show()

    spin_timer = QtCore.QTimer()

    def process_ros_callbacks() -> None:
        if rclpy.ok():
            for _callback_index in range(4):
                rclpy.spin_once(node, timeout_sec=0.0)
            node.poll_remote()

    spin_timer.timeout.connect(process_ros_callbacks)
    spin_timer.start(10)
    signal.signal(signal.SIGINT, lambda _signal, _frame: application.quit())

    try:
        exit_code = application.exec_()
    finally:
        spin_timer.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
