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

"""Detect a gray object and publish its clock-direction sector."""

from dataclasses import dataclass
import math

import numpy as np
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


# ---------------------------------------------------------------------------
# EASY-TO-TUNE BLACK / GRAY THRESHOLDS
# ---------------------------------------------------------------------------
# Pixel intensity is the average of the blue, green, and red channels.
#   0 is black; 255 is white.
#
# Pixels at or below BLACK_MAX_INTENSITY are treated as black background.
# Pixels from GRAY_MIN_INTENSITY through GRAY_MAX_INTENSITY are gray
# candidates. The gap between black_max and gray_min rejects ambiguous dark
# pixels. A gray candidate must also have its brightest and darkest color
# channels within GRAY_MAX_CHANNEL_SPREAD, which rejects strongly colored
# pixels.
DEFAULT_BLACK_MAX_INTENSITY = 45
DEFAULT_GRAY_MIN_INTENSITY = 55
DEFAULT_GRAY_MAX_INTENSITY = 220
DEFAULT_GRAY_MAX_CHANNEL_SPREAD = 30


# Figure 1 gives a 20-wide by 11-high frame. Each side of the center stem is
# 7 units wide, the stem starts 3.4 units below the top, and the excluded
# bottom strip is 2 units high. Fractions keep the ROI correct at any camera
# resolution.
DEFAULT_SIDE_VALID_WIDTH_FRACTION = 7.0 / 20.0
DEFAULT_STEM_TOP_FRACTION = 3.4 / 11.0
DEFAULT_BOTTOM_EXCLUSION_FRACTION = 2.0 / 11.0

# Figure 2 starts at 8:00, ends at 3:30, and spaces boundaries by 15 degrees.
# With zero-based numbering, 8:00-8:30 is sector 0, 8:30-9:00 is sector 1,
# and 9:00-9:30 is sector 2, matching the examples in the request.
DEFAULT_SECTOR_START_CLOCK = 8.0
DEFAULT_SECTOR_END_CLOCK = 3.5
DEFAULT_SECTOR_STEP_DEGREES = 15.0

# Reject connected gray regions smaller than this fraction of the valid ROI.
DEFAULT_MIN_OBJECT_AREA_FRACTION = 0.0005


@dataclass
class ConnectedRegion:
    """Largest connected binary region and its area centroid."""

    mask: np.ndarray
    area: int
    centroid_x: float
    centroid_y: float


@dataclass
class DetectionResult:
    """Complete gray-object result for one image."""

    mask: np.ndarray
    area: int
    centroid: tuple[float, float] | None
    sector: int


def build_valid_mask(
    height: int,
    width: int,
    side_valid_width_fraction: float =
    DEFAULT_SIDE_VALID_WIDTH_FRACTION,
    stem_top_fraction: float = DEFAULT_STEM_TOP_FRACTION,
    bottom_exclusion_fraction: float =
    DEFAULT_BOTTOM_EXCLUSION_FRACTION,
) -> np.ndarray:
    """Return the valid ROI outside the proportional red T in Figure 1."""
    if height <= 0 or width <= 0:
        raise ValueError('image height and width must be positive')
    if not 0.0 < side_valid_width_fraction < 0.5:
        raise ValueError(
            'side_valid_width_fraction must be between 0 and 0.5'
        )
    if not 0.0 <= stem_top_fraction < 1.0:
        raise ValueError('stem_top_fraction must be in [0, 1)')
    if not 0.0 < bottom_exclusion_fraction < 1.0:
        raise ValueError(
            'bottom_exclusion_fraction must be between 0 and 1'
        )
    bottom_start_fraction = 1.0 - bottom_exclusion_fraction
    if stem_top_fraction >= bottom_start_fraction:
        raise ValueError(
            'stem_top_fraction must be above the bottom exclusion'
        )

    left_stem = round(width * side_valid_width_fraction)
    right_stem = round(width * (1.0 - side_valid_width_fraction))
    stem_top = round(height * stem_top_fraction)
    bottom_top = round(height * bottom_start_fraction)

    valid = np.ones((height, width), dtype=bool)
    valid[bottom_top:, :] = False
    valid[stem_top:bottom_top, left_stem:right_stem] = False
    return valid


def threshold_gray(
    bgr_image: np.ndarray,
    valid_mask: np.ndarray,
    black_max_intensity: int = DEFAULT_BLACK_MAX_INTENSITY,
    gray_min_intensity: int = DEFAULT_GRAY_MIN_INTENSITY,
    gray_max_intensity: int = DEFAULT_GRAY_MAX_INTENSITY,
    gray_max_channel_spread: int = DEFAULT_GRAY_MAX_CHANNEL_SPREAD,
) -> np.ndarray:
    """Threshold neutral gray pixels inside the valid ROI."""
    if bgr_image.ndim != 3 or bgr_image.shape[2] != 3:
        raise ValueError('bgr_image must have shape (height, width, 3)')
    if valid_mask.shape != bgr_image.shape[:2]:
        raise ValueError('valid_mask dimensions must match bgr_image')

    blue = bgr_image[:, :, 0]
    green = bgr_image[:, :, 1]
    red = bgr_image[:, :, 2]
    intensity = (
        blue.astype(np.uint16) + green + red
    ) // 3
    maximum_channel = np.maximum(blue, green)
    np.maximum(maximum_channel, red, out=maximum_channel)
    minimum_channel = np.minimum(blue, green)
    np.minimum(minimum_channel, red, out=minimum_channel)
    channel_spread = maximum_channel - minimum_channel

    not_black = intensity > black_max_intensity
    in_gray_range = (
        (intensity >= gray_min_intensity)
        & (intensity <= gray_max_intensity)
    )
    neutral_color = channel_spread <= gray_max_channel_spread
    return valid_mask & not_black & in_gray_range & neutral_color


def denoise_binary_mask(
    mask: np.ndarray,
    minimum_neighbors: int = 5,
) -> np.ndarray:
    """Apply one 3x3 majority pass to remove isolated threshold noise."""
    if not 1 <= minimum_neighbors <= 9:
        raise ValueError('minimum_neighbors must be in [1, 9]')

    height, width = mask.shape
    padded = np.pad(mask, 1, mode='constant', constant_values=False)
    neighbors = np.zeros((height, width), dtype=np.uint8)
    for row_offset in range(3):
        for column_offset in range(3):
            neighbors += padded[
                row_offset:row_offset + height,
                column_offset:column_offset + width,
            ]
    return neighbors >= minimum_neighbors


def largest_connected_region(mask: np.ndarray) -> ConnectedRegion | None:
    """Return the largest 8-connected region using row runs."""
    if mask.ndim != 2:
        raise ValueError('mask must be a two-dimensional array')
    if not np.any(mask):
        return None

    parent: list[int] = []
    ranks: list[int] = []

    def new_label() -> int:
        label = len(parent)
        parent.append(label)
        ranks.append(0)
        return label

    def find(label: int) -> int:
        root = label
        while parent[root] != root:
            root = parent[root]
        while parent[label] != label:
            next_label = parent[label]
            parent[label] = root
            label = next_label
        return root

    def union(first: int, second: int) -> int:
        first_root = find(first)
        second_root = find(second)
        if first_root == second_root:
            return first_root
        if ranks[first_root] < ranks[second_root]:
            first_root, second_root = second_root, first_root
        parent[second_root] = first_root
        if ranks[first_root] == ranks[second_root]:
            ranks[first_root] += 1
        return first_root

    # Stored run: (row, first_column, last_column, provisional_label).
    all_runs: list[tuple[int, int, int, int]] = []
    previous_runs: list[tuple[int, int, int]] = []

    for row_index, row in enumerate(mask):
        if not np.any(row):
            previous_runs = []
            continue
        padded_row = np.pad(
            row.astype(np.int8), (1, 1), mode='constant'
        )
        transitions = np.diff(padded_row)
        starts = np.flatnonzero(transitions == 1)
        ends = np.flatnonzero(transitions == -1) - 1

        current_runs: list[tuple[int, int, int]] = []
        previous_start_index = 0
        for start, end in zip(starts.tolist(), ends.tolist()):
            while (
                previous_start_index < len(previous_runs)
                and previous_runs[previous_start_index][1] < start - 1
            ):
                previous_start_index += 1

            overlapping_labels = []
            overlap_index = previous_start_index
            while (
                overlap_index < len(previous_runs)
                and previous_runs[overlap_index][0] <= end + 1
            ):
                overlapping_labels.append(
                    previous_runs[overlap_index][2]
                )
                overlap_index += 1

            if overlapping_labels:
                label = overlapping_labels[0]
                for overlapping_label in overlapping_labels[1:]:
                    label = union(label, overlapping_label)
            else:
                label = new_label()

            current_runs.append((start, end, label))
            all_runs.append((row_index, start, end, label))

        previous_runs = current_runs

    areas: dict[int, int] = {}
    x_sums: dict[int, int] = {}
    y_sums: dict[int, int] = {}
    for row, start, end, label in all_runs:
        root = find(label)
        run_length = end - start + 1
        areas[root] = areas.get(root, 0) + run_length
        x_sums[root] = (
            x_sums.get(root, 0)
            + (start + end) * run_length // 2
        )
        y_sums[root] = y_sums.get(root, 0) + row * run_length

    largest_root = max(areas, key=areas.get)
    largest_mask = np.zeros(mask.shape, dtype=bool)
    for row, start, end, label in all_runs:
        if find(label) == largest_root:
            largest_mask[row, start:end + 1] = True

    area = areas[largest_root]
    return ConnectedRegion(
        mask=largest_mask,
        area=area,
        centroid_x=x_sums[largest_root] / area,
        centroid_y=y_sums[largest_root] / area,
    )


def clock_to_image_angle(clock_hour: float) -> float:
    """Convert a clock direction to degrees in image x-right/y-down axes."""
    return (clock_hour * 30.0 - 90.0) % 360.0


def sector_angle_range(
    start_clock: float = DEFAULT_SECTOR_START_CLOCK,
    end_clock: float = DEFAULT_SECTOR_END_CLOCK,
) -> tuple[float, float]:
    """Return an increasing image-angle range for clock boundaries."""
    start_angle = clock_to_image_angle(start_clock)
    end_angle = clock_to_image_angle(end_clock)
    while end_angle <= start_angle:
        end_angle += 360.0
    return start_angle, end_angle


def sector_for_centroid(
    centroid_x: float,
    centroid_y: float,
    center_x: float,
    center_y: float,
    start_clock: float = DEFAULT_SECTOR_START_CLOCK,
    end_clock: float = DEFAULT_SECTOR_END_CLOCK,
    step_degrees: float = DEFAULT_SECTOR_STEP_DEGREES,
) -> int:
    """Return the zero-based sector, or -1 outside the angular range."""
    delta_x = centroid_x - center_x
    delta_y = centroid_y - center_y
    if math.hypot(delta_x, delta_y) < 1.0e-9:
        return -1

    start_angle, end_angle = sector_angle_range(
        start_clock, end_clock
    )
    angle = math.degrees(math.atan2(delta_y, delta_x)) % 360.0
    while angle < start_angle:
        angle += 360.0

    if angle < start_angle or angle >= end_angle:
        return -1
    return int(math.floor((angle - start_angle) / step_degrees))


def detect_gray_object(
    bgr_image: np.ndarray,
    valid_mask: np.ndarray,
    minimum_area: int,
    black_max_intensity: int = DEFAULT_BLACK_MAX_INTENSITY,
    gray_min_intensity: int = DEFAULT_GRAY_MIN_INTENSITY,
    gray_max_intensity: int = DEFAULT_GRAY_MAX_INTENSITY,
    gray_max_channel_spread: int = DEFAULT_GRAY_MAX_CHANNEL_SPREAD,
    denoise_minimum_neighbors: int = 5,
    start_clock: float = DEFAULT_SECTOR_START_CLOCK,
    end_clock: float = DEFAULT_SECTOR_END_CLOCK,
    step_degrees: float = DEFAULT_SECTOR_STEP_DEGREES,
) -> DetectionResult:
    """Run threshold, connected-region, centroid, and sector processing."""
    threshold_mask = threshold_gray(
        bgr_image,
        valid_mask,
        black_max_intensity,
        gray_min_intensity,
        gray_max_intensity,
        gray_max_channel_spread,
    )
    cleaned_mask = denoise_binary_mask(
        threshold_mask, denoise_minimum_neighbors
    )
    cleaned_mask &= valid_mask
    region = largest_connected_region(cleaned_mask)

    empty_mask = np.zeros(valid_mask.shape, dtype=bool)
    if region is None or region.area < minimum_area:
        return DetectionResult(
            mask=empty_mask,
            area=0,
            centroid=None,
            sector=-1,
        )

    height, width = valid_mask.shape
    centroid = (region.centroid_x, region.centroid_y)
    sector = sector_for_centroid(
        region.centroid_x,
        region.centroid_y,
        (width - 1) / 2.0,
        (height - 1) / 2.0,
        start_clock,
        end_clock,
        step_degrees,
    )
    return DetectionResult(
        mask=region.mask,
        area=region.area,
        centroid=centroid,
        sector=sector,
    )


def image_message_to_bgr(message: Image) -> np.ndarray:
    """Convert common 8-bit ROS Image encodings without cv_bridge."""
    encoding = message.encoding.lower()
    channel_counts = {
        'bgr8': 3,
        'rgb8': 3,
        '8uc3': 3,
        'mono8': 1,
        '8uc1': 1,
    }
    if encoding not in channel_counts:
        raise ValueError(
            f'unsupported image encoding {message.encoding!r}; expected '
            'bgr8, rgb8, or mono8'
        )

    channels = channel_counts[encoding]
    packed_row_bytes = message.width * channels
    if message.step < packed_row_bytes:
        raise ValueError('image step is smaller than its packed row size')

    data = np.frombuffer(message.data, dtype=np.uint8)
    required_bytes = message.height * message.step
    if data.size < required_bytes:
        raise ValueError('image data is shorter than height times step')

    rows = data[:required_bytes].reshape(message.height, message.step)
    packed = rows[:, :packed_row_bytes]
    if channels == 1:
        mono = packed.reshape(message.height, message.width)
        return np.repeat(mono[:, :, np.newaxis], 3, axis=2)

    image = packed.reshape(message.height, message.width, 3)
    if encoding == 'rgb8':
        return image[:, :, ::-1].copy()
    return image.copy()


def _set_mask_color(
    image: np.ndarray,
    mask: np.ndarray,
    color: tuple[int, int, int],
) -> None:
    """Set selected BGR pixels efficiently in place."""
    for channel_index, channel_value in enumerate(color):
        image[:, :, channel_index][mask] = channel_value


def _draw_line(
    image: np.ndarray,
    start: tuple[float, float],
    end: tuple[float, float],
    color: tuple[int, int, int],
    thickness: int = 1,
) -> None:
    """Draw a clipped line on a BGR image."""
    x0, y0 = start
    x1, y1 = end
    samples = max(abs(x1 - x0), abs(y1 - y0))
    sample_count = max(1, int(math.ceil(samples)) + 1)
    x_values = np.rint(np.linspace(x0, x1, sample_count)).astype(int)
    y_values = np.rint(np.linspace(y0, y1, sample_count)).astype(int)

    radius = max(0, thickness // 2)
    height, width = image.shape[:2]
    for y_offset in range(-radius, radius + 1):
        for x_offset in range(-radius, radius + 1):
            x_pixels = x_values + x_offset
            y_pixels = y_values + y_offset
            inside = (
                (x_pixels >= 0)
                & (x_pixels < width)
                & (y_pixels >= 0)
                & (y_pixels < height)
            )
            image[y_pixels[inside], x_pixels[inside]] = color


def _draw_circle(
    image: np.ndarray,
    center: tuple[float, float],
    radius: int,
    color: tuple[int, int, int],
    filled: bool = False,
) -> None:
    """Draw a circle on a BGR image."""
    center_x = int(round(center[0]))
    center_y = int(round(center[1]))
    height, width = image.shape[:2]
    x_min = max(0, center_x - radius - 1)
    x_max = min(width - 1, center_x + radius + 1)
    y_min = max(0, center_y - radius - 1)
    y_max = min(height - 1, center_y + radius + 1)
    if x_min > x_max or y_min > y_max:
        return

    y_grid, x_grid = np.ogrid[y_min:y_max + 1, x_min:x_max + 1]
    distance_squared = (
        (x_grid - center_x) ** 2 + (y_grid - center_y) ** 2
    )
    if filled:
        circle = distance_squared <= radius ** 2
    else:
        inner_radius = max(0, radius - 2)
        circle = (
            (distance_squared <= radius ** 2)
            & (distance_squared >= inner_radius ** 2)
        )
    target = image[y_min:y_max + 1, x_min:x_max + 1]
    target[circle] = color


_FONT_5X7 = {
    ' ': ('00000',) * 7,
    '-': (
        '00000', '00000', '00000', '11111', '00000', '00000', '00000',
    ),
    '0': (
        '01110', '10001', '10011', '10101', '11001', '10001', '01110',
    ),
    '1': (
        '00100', '01100', '00100', '00100', '00100', '00100', '01110',
    ),
    '2': (
        '01110', '10001', '00001', '00010', '00100', '01000', '11111',
    ),
    '3': (
        '11110', '00001', '00001', '01110', '00001', '00001', '11110',
    ),
    '4': (
        '00010', '00110', '01010', '10010', '11111', '00010', '00010',
    ),
    '5': (
        '11111', '10000', '10000', '11110', '00001', '00001', '11110',
    ),
    '6': (
        '01110', '10000', '10000', '11110', '10001', '10001', '01110',
    ),
    '7': (
        '11111', '00001', '00010', '00100', '01000', '01000', '01000',
    ),
    '8': (
        '01110', '10001', '10001', '01110', '10001', '10001', '01110',
    ),
    '9': (
        '01110', '10001', '10001', '01111', '00001', '00001', '01110',
    ),
    'C': (
        '01111', '10000', '10000', '10000', '10000', '10000', '01111',
    ),
    'E': (
        '11111', '10000', '10000', '11110', '10000', '10000', '11111',
    ),
    'O': (
        '01110', '10001', '10001', '10001', '10001', '10001', '01110',
    ),
    'R': (
        '11110', '10001', '10001', '11110', '10100', '10010', '10001',
    ),
    'S': (
        '01111', '10000', '10000', '01110', '00001', '00001', '11110',
    ),
    'T': (
        '11111', '00100', '00100', '00100', '00100', '00100', '00100',
    ),
}


def _draw_text(
    image: np.ndarray,
    text: str,
    origin: tuple[int, int],
    color: tuple[int, int, int],
    scale: int,
) -> None:
    """Draw the small built-in 5x7 font without OpenCV."""
    origin_x, origin_y = origin
    height, width = image.shape[:2]
    for character_index, character in enumerate(text):
        glyph = _FONT_5X7.get(character, _FONT_5X7[' '])
        character_x = origin_x + character_index * 6 * scale
        for glyph_y, row in enumerate(glyph):
            for glyph_x, enabled in enumerate(row):
                if enabled != '1':
                    continue
                x0 = character_x + glyph_x * scale
                y0 = origin_y + glyph_y * scale
                x1 = min(width, x0 + scale)
                y1 = min(height, y0 + scale)
                if x0 < width and y0 < height and x1 > 0 and y1 > 0:
                    image[max(0, y0):y1, max(0, x0):x1] = color


def _ray_endpoint(
    center: tuple[float, float],
    angle_degrees: float,
    width: int,
    height: int,
) -> tuple[float, float]:
    """Find where a ray first meets an image edge."""
    center_x, center_y = center
    angle_radians = math.radians(angle_degrees)
    delta_x = math.cos(angle_radians)
    delta_y = math.sin(angle_radians)
    distances = []
    if delta_x > 1.0e-9:
        distances.append((width - 1 - center_x) / delta_x)
    elif delta_x < -1.0e-9:
        distances.append((0.0 - center_x) / delta_x)
    if delta_y > 1.0e-9:
        distances.append((height - 1 - center_y) / delta_y)
    elif delta_y < -1.0e-9:
        distances.append((0.0 - center_y) / delta_y)
    distance = min(value for value in distances if value >= 0.0)
    return (
        center_x + distance * delta_x,
        center_y + distance * delta_y,
    )


def make_debug_image(
    bgr_image: np.ndarray,
    valid_mask: np.ndarray,
    detected_mask: np.ndarray,
    centroid: tuple[float, float] | None,
    sector: int,
    start_clock: float = DEFAULT_SECTOR_START_CLOCK,
    end_clock: float = DEFAULT_SECTOR_END_CLOCK,
    step_degrees: float = DEFAULT_SECTOR_STEP_DEGREES,
    hatch_mask: np.ndarray | None = None,
) -> np.ndarray:
    """Render ROI, detection, centroid, center, rays, and sector label."""
    debug = bgr_image.copy()
    height, width = debug.shape[:2]
    center = ((width - 1) / 2.0, (height - 1) / 2.0)

    # Valid pixels retain the camera image. Excluded pixels are dark red and
    # hatched so the proportional T-shaped exclusion is unmistakable.
    invalid_mask = ~valid_mask
    _set_mask_color(debug, invalid_mask, (0, 0, 65))
    if hatch_mask is None:
        hatch_spacing = max(10, min(width, height) // 24)
        row_positions = np.arange(height)[:, np.newaxis]
        column_positions = np.arange(width)[np.newaxis, :]
        hatch_mask = (
            invalid_mask
            & (
                (
                    row_positions + column_positions
                ) % hatch_spacing < 2
            )
        )
    _set_mask_color(debug, hatch_mask, (0, 0, 255))

    # Selected largest connected gray region.
    _set_mask_color(debug, detected_mask, (0, 220, 0))

    start_angle, end_angle = sector_angle_range(
        start_clock, end_clock
    )
    boundary_count = int(round(
        (end_angle - start_angle) / step_degrees
    ))
    for boundary_index in range(boundary_count + 1):
        boundary_angle = start_angle + boundary_index * step_degrees
        endpoint = _ray_endpoint(
            center, boundary_angle, width, height
        )
        _draw_line(debug, center, endpoint, (255, 0, 255), 2)

    # Highlight the two boundaries around the selected sector.
    if sector >= 0:
        for boundary_angle in (
            start_angle + sector * step_degrees,
            start_angle + (sector + 1) * step_degrees,
        ):
            endpoint = _ray_endpoint(
                center, boundary_angle, width, height
            )
            _draw_line(debug, center, endpoint, (0, 255, 255), 3)

    cross_size = max(8, min(width, height) // 45)
    _draw_circle(debug, center, cross_size // 2, (255, 255, 0), True)
    _draw_line(
        debug,
        (center[0] - cross_size, center[1]),
        (center[0] + cross_size, center[1]),
        (255, 255, 0),
        2,
    )
    _draw_line(
        debug,
        (center[0], center[1] - cross_size),
        (center[0], center[1] + cross_size),
        (255, 255, 0),
        2,
    )

    if centroid is not None:
        radius = max(7, min(width, height) // 55)
        _draw_circle(debug, centroid, radius, (0, 255, 255), False)
        _draw_line(
            debug,
            (centroid[0] - radius, centroid[1]),
            (centroid[0] + radius, centroid[1]),
            (0, 255, 255),
            2,
        )
        _draw_line(
            debug,
            (centroid[0], centroid[1] - radius),
            (centroid[0], centroid[1] + radius),
            (0, 255, 255),
            2,
        )

    label = f'SECTOR {sector}'
    scale = max(1, min(width, height) // 220)
    label_width = len(label) * 6 * scale + 2 * scale
    label_height = 9 * scale
    x_end = min(width, scale + label_width)
    y_end = min(height, scale + label_height)
    debug[scale:y_end, scale:x_end] = (0, 0, 0)
    label_color = (0, 255, 0) if sector >= 0 else (0, 0, 255)
    _draw_text(
        debug,
        label,
        (2 * scale, 2 * scale),
        label_color,
        scale,
    )
    return debug


class GrayObjectDetector(Node):
    """Detect the largest gray region and publish its radial sector."""

    def __init__(self) -> None:
        super().__init__('gray_object_detector')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter(
            'debug_image_topic', '/gray_object/debug_image'
        )
        self.declare_parameter('sector_topic', '/gray_object/sector')
        self.declare_parameter('publish_debug_image', True)

        self.declare_parameter(
            'black_max_intensity', DEFAULT_BLACK_MAX_INTENSITY
        )
        self.declare_parameter(
            'gray_min_intensity', DEFAULT_GRAY_MIN_INTENSITY
        )
        self.declare_parameter(
            'gray_max_intensity', DEFAULT_GRAY_MAX_INTENSITY
        )
        self.declare_parameter(
            'gray_max_channel_spread',
            DEFAULT_GRAY_MAX_CHANNEL_SPREAD,
        )
        self.declare_parameter('denoise_minimum_neighbors', 5)
        self.declare_parameter(
            'min_object_area_fraction',
            DEFAULT_MIN_OBJECT_AREA_FRACTION,
        )

        self.declare_parameter(
            'side_valid_width_fraction',
            DEFAULT_SIDE_VALID_WIDTH_FRACTION,
        )
        self.declare_parameter(
            'stem_top_fraction', DEFAULT_STEM_TOP_FRACTION
        )
        self.declare_parameter(
            'bottom_exclusion_fraction',
            DEFAULT_BOTTOM_EXCLUSION_FRACTION,
        )

        self.declare_parameter(
            'sector_start_clock', DEFAULT_SECTOR_START_CLOCK
        )
        self.declare_parameter(
            'sector_end_clock', DEFAULT_SECTOR_END_CLOCK
        )
        self.declare_parameter(
            'sector_step_degrees', DEFAULT_SECTOR_STEP_DEGREES
        )

        self._black_max = self.get_parameter(
            'black_max_intensity'
        ).value
        self._gray_min = self.get_parameter(
            'gray_min_intensity'
        ).value
        self._gray_max = self.get_parameter(
            'gray_max_intensity'
        ).value
        self._gray_spread = self.get_parameter(
            'gray_max_channel_spread'
        ).value
        self._denoise_neighbors = self.get_parameter(
            'denoise_minimum_neighbors'
        ).value
        self._minimum_area_fraction = self.get_parameter(
            'min_object_area_fraction'
        ).value
        self._side_width_fraction = self.get_parameter(
            'side_valid_width_fraction'
        ).value
        self._stem_top_fraction = self.get_parameter(
            'stem_top_fraction'
        ).value
        self._bottom_fraction = self.get_parameter(
            'bottom_exclusion_fraction'
        ).value
        self._start_clock = self.get_parameter(
            'sector_start_clock'
        ).value
        self._end_clock = self.get_parameter(
            'sector_end_clock'
        ).value
        self._step_degrees = self.get_parameter(
            'sector_step_degrees'
        ).value
        self._publish_debug = self.get_parameter(
            'publish_debug_image'
        ).value
        self._validate_parameters()

        image_topic = self.get_parameter('image_topic').value
        sector_topic = self.get_parameter('sector_topic').value
        debug_topic = self.get_parameter('debug_image_topic').value
        self._sector_publisher = self.create_publisher(
            Int32, sector_topic, 10
        )
        self._debug_publisher = self.create_publisher(
            Image, debug_topic, qos_profile_sensor_data
        )
        latest_image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscription = self.create_subscription(
            Image,
            image_topic,
            self._on_image,
            latest_image_qos,
        )

        self._cached_shape: tuple[int, int] | None = None
        self._valid_mask: np.ndarray | None = None
        self._hatch_mask: np.ndarray | None = None
        self._valid_area = 0
        self._last_sector: int | None = None
        self._reported_image_error = False

        self.get_logger().info(
            f'Listening for {image_topic}; publishing sector on '
            f'{sector_topic} and debug image on {debug_topic}'
        )
        self.get_logger().info(
            'Gray thresholds: black <= '
            f'{self._black_max}, gray {self._gray_min}..'
            f'{self._gray_max}, channel spread <= {self._gray_spread}'
        )

    def _validate_parameters(self) -> None:
        threshold_values = (
            self._black_max,
            self._gray_min,
            self._gray_max,
            self._gray_spread,
        )
        if not all(
            isinstance(value, int) and 0 <= value <= 255
            for value in threshold_values
        ):
            raise ValueError(
                'intensity and channel thresholds must be integers in '
                '[0, 255]'
            )
        if not self._black_max < self._gray_min <= self._gray_max:
            raise ValueError(
                'thresholds must satisfy black_max < gray_min <= gray_max'
            )
        if not (
            isinstance(self._denoise_neighbors, int)
            and 1 <= self._denoise_neighbors <= 9
        ):
            raise ValueError(
                'denoise_minimum_neighbors must be an integer in [1, 9]'
            )
        if not 0.0 <= self._minimum_area_fraction < 1.0:
            raise ValueError(
                'min_object_area_fraction must be in [0, 1)'
            )

        # Also validates all ROI relationships.
        build_valid_mask(
            11,
            20,
            self._side_width_fraction,
            self._stem_top_fraction,
            self._bottom_fraction,
        )
        if self._step_degrees <= 0.0:
            raise ValueError('sector_step_degrees must be positive')
        start_angle, end_angle = sector_angle_range(
            self._start_clock, self._end_clock
        )
        sector_count = (end_angle - start_angle) / self._step_degrees
        if not math.isclose(
            sector_count, round(sector_count), abs_tol=1.0e-6
        ):
            raise ValueError(
                'clock range must contain a whole number of sector steps'
            )

    def _geometry_for_shape(
        self,
        height: int,
        width: int,
    ) -> np.ndarray:
        shape = (height, width)
        if self._cached_shape != shape:
            self._valid_mask = build_valid_mask(
                height,
                width,
                self._side_width_fraction,
                self._stem_top_fraction,
                self._bottom_fraction,
            )
            self._valid_area = int(np.count_nonzero(self._valid_mask))
            hatch_spacing = max(10, min(width, height) // 24)
            row_positions = np.arange(height)[:, np.newaxis]
            column_positions = np.arange(width)[np.newaxis, :]
            self._hatch_mask = (
                ~self._valid_mask
                & (
                    (
                        row_positions + column_positions
                    ) % hatch_spacing < 2
                )
            )
            self._cached_shape = shape
            self.get_logger().info(
                f'ROI configured for {width}x{height}: '
                f'{self._valid_area} valid pixels'
            )
        return self._valid_mask

    def _on_image(self, message: Image) -> None:
        try:
            bgr_image = image_message_to_bgr(message)
        except ValueError as error:
            if not self._reported_image_error:
                self.get_logger().error(str(error))
                self._reported_image_error = True
            return

        height, width = bgr_image.shape[:2]
        valid_mask = self._geometry_for_shape(height, width)
        minimum_area = max(
            1,
            math.ceil(
                self._valid_area * self._minimum_area_fraction
            ),
        )
        result = detect_gray_object(
            bgr_image,
            valid_mask,
            minimum_area,
            self._black_max,
            self._gray_min,
            self._gray_max,
            self._gray_spread,
            self._denoise_neighbors,
            self._start_clock,
            self._end_clock,
            self._step_degrees,
        )

        sector_message = Int32()
        sector_message.data = result.sector
        self._sector_publisher.publish(sector_message)

        if result.sector != self._last_sector:
            if result.centroid is None:
                self.get_logger().info(
                    f'No gray object (minimum area {minimum_area}); '
                    'publishing sector -1'
                )
            else:
                self.get_logger().info(
                    f'Gray object area={result.area}, centroid='
                    f'({result.centroid[0]:.1f}, '
                    f'{result.centroid[1]:.1f}), '
                    f'sector={result.sector}'
                )
            self._last_sector = result.sector

        if self._publish_debug:
            debug = make_debug_image(
                bgr_image,
                valid_mask,
                result.mask,
                result.centroid,
                result.sector,
                self._start_clock,
                self._end_clock,
                self._step_degrees,
                self._hatch_mask,
            )
            debug_message = Image()
            debug_message.header = message.header
            debug_message.height = height
            debug_message.width = width
            debug_message.encoding = 'bgr8'
            debug_message.is_bigendian = 0
            debug_message.step = width * 3
            debug_message.data = debug.tobytes()
            self._debug_publisher.publish(debug_message)


def main(args=None) -> None:
    """Run the gray object detector."""
    rclpy.init(args=args)
    node = None
    try:
        node = GrayObjectDetector()
        rclpy.spin(node)
    except (RuntimeError, ValueError) as error:
        if node is None:
            get_logger('gray_object_detector_startup').fatal(str(error))
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
