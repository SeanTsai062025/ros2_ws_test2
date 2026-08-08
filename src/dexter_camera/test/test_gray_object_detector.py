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

"""Tests for full-frame color detection and sector mapping."""

import math
from unittest.mock import patch

from dexter_camera.gray_object_detector import AreaTarget
from dexter_camera.gray_object_detector import LabeledDetection
from dexter_camera.gray_object_detector import RADIAL_LINE_COLOR
from dexter_camera.gray_object_detector import build_full_frame_mask
from dexter_camera.gray_object_detector import denoise_binary_mask
from dexter_camera.gray_object_detector import detect_gray_object
from dexter_camera.gray_object_detector import detect_labeled_objects
from dexter_camera.gray_object_detector import largest_connected_region
from dexter_camera.gray_object_detector import make_debug_image
from dexter_camera.gray_object_detector import orient_bgr_image
from dexter_camera.gray_object_detector import sector_for_centroid
from dexter_camera.gray_object_detector import threshold_gray
from dexter_camera.gray_object_detector import threshold_rgb
from dexter_camera.gray_object_detector import validate_area_thresholds
from dexter_camera.gray_object_detector import validate_area_targets
from dexter_camera.gray_object_detector import validate_gray_thresholds
from dexter_camera.gray_object_detector import validate_rgb_thresholds

import numpy as np


def _point_at_angle(
    angle_degrees: float,
    radius: float = 40.0,
) -> tuple[float, float]:
    angle = math.radians(angle_degrees)
    return radius * math.cos(angle), radius * math.sin(angle)


def test_full_frame_mask_has_no_excluded_pixels():
    mask = build_full_frame_mask(720, 1280)

    assert mask.shape == (720, 1280)
    assert np.all(mask)
    assert np.count_nonzero(mask) == 1280 * 720


def test_vertical_orientation_swaps_top_and_bottom_rows():
    image = np.array([
        [[1, 2, 3], [4, 5, 6]],
        [[7, 8, 9], [10, 11, 12]],
        [[13, 14, 15], [16, 17, 18]],
    ], dtype=np.uint8)

    flipped = orient_bgr_image(image, True)

    assert np.array_equal(flipped[0], image[-1])
    assert np.array_equal(flipped[-1], image[0])
    assert orient_bgr_image(image, False) is image
    flipped[0, 0] = 0
    assert np.array_equal(image[-1, 0], [13, 14, 15])


def test_horizontal_orientation_swaps_left_and_right_columns():
    image = np.array([
        [[1, 2, 3], [4, 5, 6]],
        [[7, 8, 9], [10, 11, 12]],
    ], dtype=np.uint8)

    horizontal = orient_bgr_image(image, False, True)
    rotated = orient_bgr_image(image, True, True)

    assert np.array_equal(horizontal[:, 0], image[:, -1])
    assert np.array_equal(horizontal[:, -1], image[:, 0])
    assert np.array_equal(rotated[0, 0], image[-1, -1])
    assert np.array_equal(rotated[-1, -1], image[0, 0])


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


def test_live_threshold_validation_accepts_only_ordered_rgb_values():
    validate_gray_thresholds(45, 55, 225, 30)

    with np.testing.assert_raises_regex(
        ValueError, 'black_max < gray_min'
    ):
        validate_gray_thresholds(55, 55, 225, 30)
    with np.testing.assert_raises_regex(ValueError, r'\[0, 255\]'):
        validate_gray_thresholds(45, 55, 256, 30)


def test_direct_rgb_thresholds_are_independent_and_inclusive():
    image = np.array([[
        [100, 150, 200],
        [100, 150, 221],
        [100, 171, 200],
        [79, 150, 200],
    ]], dtype=np.uint8)
    valid = np.ones((1, 4), dtype=bool)

    detected = threshold_rgb(
        image,
        valid,
        red_min=180,
        red_max=220,
        green_min=130,
        green_max=170,
        blue_min=80,
        blue_max=120,
    )

    assert detected.tolist() == [[True, False, False, False]]
    validate_rgb_thresholds(0, 255, 20, 40, 100, 100)
    with np.testing.assert_raises_regex(ValueError, 'minimum'):
        validate_rgb_thresholds(20, 10, 0, 255, 0, 255)

    validate_area_thresholds(14543, 1500)
    with np.testing.assert_raises_regex(ValueError, 'non-negative'):
        validate_area_thresholds(14543, -1)


def test_named_area_ranges_must_not_overlap():
    validate_area_targets((
        AreaTarget('little guy', 15850, 1500),
        AreaTarget('big guy', 33850, 3000),
        AreaTarget('cup', 93300, 4000),
    ))

    with np.testing.assert_raises_regex(ValueError, 'must not overlap'):
        validate_area_targets((
            AreaTarget('little guy', 15000, 5000),
            AreaTarget('big guy', 23000, 4000),
        ))


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


def test_area_filter_selects_target_instead_of_larger_same_color_object():
    image = np.zeros((80, 160, 3), dtype=np.uint8)
    image[10:20, 10:22] = (100, 100, 100)
    image[40:60, 100:130] = (100, 100, 100)
    valid = np.ones((80, 160), dtype=bool)

    unfiltered = detect_gray_object(
        image,
        valid,
        minimum_area=1,
        denoise_minimum_neighbors=1,
    )
    filtered = detect_gray_object(
        image,
        valid,
        minimum_area=1,
        denoise_minimum_neighbors=1,
        target_area=168,
        area_tolerance=0,
    )

    assert unfiltered.area == 704
    assert unfiltered.centroid[0] > 100
    assert filtered.area == 168
    assert filtered.candidate_area == 168
    assert filtered.centroid[0] < 30
    assert filtered.bounding_box == (9, 9, 22, 20)


def test_full_circle_sector_numbering_is_clockwise_from_twelve():
    center_x = 100.0
    center_y = 100.0

    expected_sectors = {
        277.5: 0,   # 12:00..12:30
        7.5: 6,     # 3:00..3:30
        97.5: 12,   # 6:00..6:30
        187.5: 18,  # 9:00..9:30
        262.5: 23,  # 11:30..12:00
    }
    for angle, expected_sector in expected_sectors.items():
        offset_x, offset_y = _point_at_angle(angle)
        assert sector_for_centroid(
            center_x + offset_x,
            center_y + offset_y,
            center_x,
            center_y,
        ) == expected_sector


def test_lower_half_is_numbered_and_center_returns_minus_one():
    center_x = 100.0
    center_y = 100.0
    offset_x, offset_y = _point_at_angle(90.0)

    assert sector_for_centroid(
        center_x + offset_x,
        center_y + offset_y,
        center_x,
        center_y,
    ) == 12
    assert sector_for_centroid(
        center_x,
        center_y,
        center_x,
        center_y,
    ) == -1


def test_debug_grid_draws_pastel_rays_without_wedge_numbers():
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    valid = np.ones((240, 320), dtype=bool)
    empty = np.zeros((240, 320), dtype=bool)

    debug = make_debug_image(
        image,
        valid,
        empty,
        centroid=None,
        sector=-1,
    )

    bottom_center = debug[-1, 158:162]
    assert np.any(np.all(bottom_center == RADIAL_LINE_COLOR, axis=1))
    white_label_pixels = np.all(debug == (255, 255, 255), axis=2)
    assert np.count_nonzero(white_label_pixels) == 0


def test_named_boxes_show_xy_but_only_little_guy_shows_section():
    image = np.zeros((100, 160, 3), dtype=np.uint8)
    empty = np.zeros((100, 160), dtype=bool)
    detections = (
        LabeledDetection('cup', empty, 90000, (130.4, 20.6),
                         (120, 10, 150, 40), 4),
        LabeledDetection('big guy', empty, 33000, (80.2, 30.2),
                         (70, 20, 90, 40), 21),
        LabeledDetection('little guy', empty, 16000, (20.6, 25.4),
                         (10, 15, 30, 35), 19),
    )

    with patch(
        'dexter_camera.gray_object_detector._draw_text'
    ) as draw_text:
        make_debug_image(
            image,
            np.ones((100, 160), dtype=bool),
            empty,
            centroid=None,
            sector=19,
            labeled_detections=detections,
        )

    rendered_text = [call.args[1] for call in draw_text.call_args_list]
    assert 'CUP' in rendered_text
    assert 'X 130 Y 21' in rendered_text
    assert 'BIG GUY' in rendered_text
    assert 'X 80 Y 30' in rendered_text
    assert 'LITTLE GUY' in rendered_text
    assert 'X 21 Y 25' in rendered_text
    assert 'LITTLE GUY SECTION 19' in rendered_text
    assert not any(text.startswith('CUP SECTION') for text in rendered_text)
    assert not any(
        text.startswith('BIG GUY SECTION') for text in rendered_text
    )


def test_all_named_area_ranges_are_detected_in_the_same_frame():
    image = np.zeros((120, 240, 3), dtype=np.uint8)
    image[15:25, 10:22] = (100, 100, 100)
    image[35:50, 90:115] = (100, 100, 100)
    image[60:80, 170:210] = (100, 100, 100)
    valid = build_full_frame_mask(120, 240)
    targets = (
        AreaTarget('cup', 924, 0),
        AreaTarget('big guy', 459, 0),
        AreaTarget('little guy', 168, 0),
    )

    result = detect_labeled_objects(
        image,
        valid,
        minimum_area=1,
        area_targets=targets,
        denoise_minimum_neighbors=1,
        red_min=90,
        red_max=110,
        green_min=90,
        green_max=110,
        blue_min=90,
        blue_max=110,
    )

    assert [item.label for item in result.detections] == [
        'cup',
        'big guy',
        'little guy',
    ]
    assert [item.area for item in result.detections] == [924, 459, 168]
    assert all(item.bounding_box for item in result.detections)
    assert all(0 <= item.sector < 24 for item in result.detections)


def test_every_region_matching_one_named_range_is_returned():
    image = np.zeros((80, 140, 3), dtype=np.uint8)
    image[10:20, 10:22] = (100, 100, 100)
    image[45:55, 100:112] = (100, 100, 100)
    valid = build_full_frame_mask(80, 140)

    result = detect_labeled_objects(
        image,
        valid,
        minimum_area=1,
        area_targets=(AreaTarget('little guy', 168, 0),),
        denoise_minimum_neighbors=1,
        red_min=90,
        red_max=110,
        green_min=90,
        green_max=110,
        blue_min=90,
        blue_max=110,
    )

    assert len(result.detections) == 2
    assert all(item.label == 'little guy' for item in result.detections)
    assert all(item.area == 168 for item in result.detections)


def test_full_detection_path_returns_centroid_and_sector():
    image = np.full((110, 200, 3), 10, dtype=np.uint8)
    image[35:50, 20:50] = (110, 112, 108)
    valid = build_full_frame_mask(110, 200)

    result = detect_gray_object(image, valid, minimum_area=100)

    assert result.area > 400
    assert result.centroid is not None
    assert 33.0 < result.centroid[0] < 36.0
    assert 41.0 < result.centroid[1] < 44.0
    assert result.sector == 18
    assert result.mask[42, 34]
    assert result.candidate_area == result.area
    assert result.bounding_box is not None

    rejected_by_area = detect_gray_object(
        image,
        valid,
        minimum_area=100,
        target_area=result.area + 1000,
        area_tolerance=10,
    )
    assert rejected_by_area.area == 0
    assert rejected_by_area.candidate_area == result.area
    assert rejected_by_area.centroid is None
    assert rejected_by_area.bounding_box is None

    hatch = np.zeros(valid.shape, dtype=bool)
    debug = make_debug_image(
        image,
        valid,
        result.mask,
        result.centroid,
        result.sector,
        hatch_mask=hatch,
        candidate_mask=result.candidate_mask,
        bounding_box=result.bounding_box,
    )
    assert debug.shape == image.shape
    left, top, _, _ = result.bounding_box
    assert tuple(debug[top, left]) == (0, 255, 255)


def test_former_center_and_bottom_exclusions_are_fully_detected():
    valid = build_full_frame_mask(110, 200)
    former_exclusion_objects = (
        (slice(45, 60), slice(85, 115)),
        (slice(95, 108), slice(20, 55)),
    )

    for rows, columns in former_exclusion_objects:
        image = np.zeros((110, 200, 3), dtype=np.uint8)
        image[rows, columns] = (100, 100, 100)
        candidate = detect_gray_object(
            image,
            valid,
            minimum_area=1,
            denoise_minimum_neighbors=1,
            red_min=90,
            red_max=110,
            green_min=90,
            green_max=110,
            blue_min=90,
            blue_max=110,
        )
        locked = detect_gray_object(
            image,
            valid,
            minimum_area=1,
            denoise_minimum_neighbors=1,
            red_min=90,
            red_max=110,
            green_min=90,
            green_max=110,
            blue_min=90,
            blue_max=110,
            target_area=candidate.area,
            area_tolerance=0,
        )

        assert candidate.area > 0
        assert locked.area == candidate.area
        assert locked.candidate_area == candidate.area
        assert locked.centroid is not None
        assert locked.bounding_box is not None
        assert 0 <= locked.sector < 24
