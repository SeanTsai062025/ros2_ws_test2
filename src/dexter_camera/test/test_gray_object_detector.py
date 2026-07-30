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

"""Tests for proportional ROI, gray detection, and sector mapping."""

import math

from dexter_camera.gray_object_detector import build_valid_mask
from dexter_camera.gray_object_detector import denoise_binary_mask
from dexter_camera.gray_object_detector import detect_gray_object
from dexter_camera.gray_object_detector import largest_connected_region
from dexter_camera.gray_object_detector import make_debug_image
from dexter_camera.gray_object_detector import sector_for_centroid
from dexter_camera.gray_object_detector import threshold_gray

import numpy as np


def _point_at_angle(
    angle_degrees: float,
    radius: float = 40.0,
) -> tuple[float, float]:
    angle = math.radians(angle_degrees)
    return radius * math.cos(angle), radius * math.sin(angle)


def test_valid_mask_matches_figure_one_proportions():
    mask = build_valid_mask(110, 200)

    assert mask[0, 100]
    assert mask[33, 100]
    assert not mask[34, 100]
    assert mask[50, 69]
    assert not mask[50, 70]
    assert mask[50, 130]
    assert mask[89, 10]
    assert not mask[90, 10]


def test_threshold_gray_rejects_black_color_and_excluded_pixels():
    image = np.zeros((4, 5, 3), dtype=np.uint8)
    image[0, 0] = (100, 100, 100)
    image[0, 1] = (40, 40, 40)
    image[0, 2] = (40, 40, 120)
    image[0, 3] = (100, 100, 100)
    valid = np.ones((4, 5), dtype=bool)
    valid[0, 3] = False

    detected = threshold_gray(image, valid)

    assert detected[0, 0]
    assert not detected[0, 1]
    assert not detected[0, 2]
    assert not detected[0, 3]


def test_denoise_and_largest_region_choose_object_by_area():
    mask = np.zeros((20, 30), dtype=bool)
    mask[2:10, 3:12] = True
    mask[14:17, 22:25] = True
    mask[0, 29] = True

    cleaned = denoise_binary_mask(mask)
    region = largest_connected_region(cleaned)

    assert region is not None
    assert region.area > 50
    assert 6.0 < region.centroid_x < 8.0
    assert 4.0 < region.centroid_y < 7.0
    assert not region.mask[15, 23]


def test_sector_numbering_matches_request_examples():
    center_x = 100.0
    center_y = 100.0

    # 8:30..9:00 corresponds to image angles 165..180 degrees.
    offset_x, offset_y = _point_at_angle(172.5)
    assert sector_for_centroid(
        center_x + offset_x,
        center_y + offset_y,
        center_x,
        center_y,
    ) == 1

    # 9:00..9:30 corresponds to image angles 180..195 degrees.
    offset_x, offset_y = _point_at_angle(187.5)
    assert sector_for_centroid(
        center_x + offset_x,
        center_y + offset_y,
        center_x,
        center_y,
    ) == 2


def test_sector_range_and_center_return_minus_one():
    center_x = 100.0
    center_y = 100.0
    offset_x, offset_y = _point_at_angle(90.0)

    assert sector_for_centroid(
        center_x + offset_x,
        center_y + offset_y,
        center_x,
        center_y,
    ) == -1
    assert sector_for_centroid(
        center_x,
        center_y,
        center_x,
        center_y,
    ) == -1


def test_full_detection_path_returns_centroid_and_sector():
    image = np.full((110, 200, 3), 10, dtype=np.uint8)
    image[35:50, 20:50] = (110, 112, 108)
    valid = build_valid_mask(110, 200)

    result = detect_gray_object(image, valid, minimum_area=100)

    assert result.area > 400
    assert result.centroid is not None
    assert 33.0 < result.centroid[0] < 36.0
    assert 41.0 < result.centroid[1] < 44.0
    assert result.sector == 2
    assert result.mask[42, 34]

    hatch = np.zeros(valid.shape, dtype=bool)
    debug = make_debug_image(
        image,
        valid,
        result.mask,
        result.centroid,
        result.sector,
        hatch_mask=hatch,
    )
    assert debug.shape == image.shape
