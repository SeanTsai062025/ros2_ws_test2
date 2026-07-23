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

"""Tests for camera pipeline and calibration helpers."""

from pathlib import Path

from dexter_camera.camera_node import build_pipeline
from dexter_camera.camera_node import load_camera_info

import pytest


def test_build_libcamera_pipeline():
    pipeline = build_pipeline(
        'libcamera', 1280, 720, 30.0, 'camera "zero"'
    )

    assert pipeline.startswith('libcamerasrc name=source')
    assert 'width=1280,height=720,framerate=30/1' in pipeline
    assert 'camera-name="camera \\"zero\\""' in pipeline
    assert 'appsink name=sink' in pipeline


def test_build_fractional_rate_test_pipeline():
    pipeline = build_pipeline('test', 640, 480, 29.97)

    assert pipeline.startswith('videotestsrc is-live=true')
    assert 'framerate=2997/100' in pipeline


def test_reject_unknown_source():
    with pytest.raises(ValueError, match='Unsupported source'):
        build_pipeline('usb', 640, 480, 30.0)


def test_load_camera_info(tmp_path: Path):
    calibration = tmp_path / 'camera.yaml'
    calibration.write_text(
        '\n'.join([
            'image_width: 640',
            'image_height: 480',
            'camera_name: test',
            'camera_matrix:',
            '  data: [1, 0, 2, 0, 3, 4, 0, 0, 1]',
            'distortion_model: plumb_bob',
            'distortion_coefficients:',
            '  data: [0, 0, 0, 0, 0]',
            'rectification_matrix:',
            '  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]',
            'projection_matrix:',
            '  data: [1, 0, 2, 0, 0, 3, 4, 0, 0, 0, 1, 0]',
        ]),
        encoding='utf-8',
    )

    loaded = load_camera_info(f'file://{calibration}')

    assert loaded is not None
    assert loaded['camera_name'] == 'test'


def test_reject_bad_calibration(tmp_path: Path):
    calibration = tmp_path / 'bad.yaml'
    calibration.write_text(
        'camera_matrix:\n  data: [1, 2]\n',
        encoding='utf-8',
    )

    with pytest.raises(ValueError, match='camera_matrix.data'):
        load_camera_info(str(calibration))
