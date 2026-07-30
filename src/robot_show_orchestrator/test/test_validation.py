from copy import deepcopy
from pathlib import Path

import pytest

from robot_show_orchestrator.validation import (
    load_show,
    ShowValidationError,
    validate_show,
)
import yaml


DEMO = Path(__file__).parents[1] / 'config' / 'demo_show.yaml'


def read_demo():
    return yaml.safe_load(DEMO.read_text(encoding='utf-8'))


def test_demo_show_is_valid():
    document = load_show(str(DEMO))
    assert document['show']['id'] == 'robot_table_demo'


def test_duplicate_node_ids_are_rejected():
    document = read_demo()
    branches = document['show']['root']['children'][0]['branches']
    branches[1]['id'] = branches[0]['id']
    with pytest.raises(ShowValidationError, match='duplicate node id'):
        validate_show(document)


def test_parallel_same_device_is_rejected():
    document = read_demo()
    parallel = document['show']['root']['children'][0]
    duplicate_arm = deepcopy(parallel['branches'][0])
    duplicate_arm['id'] = 'second_arm_branch'
    duplicate_arm['children'][0]['id'] = 'second_arm_action'
    parallel['branches'].append(duplicate_arm)
    with pytest.raises(ShowValidationError, match='parallel resource conflict'):
        validate_show(document)


def test_illegal_rail_direction_is_rejected():
    document = read_demo()
    rail_branch = document['show']['root']['children'][0]['branches'][2]
    rail_branch['children'][0]['branches'][0]['args']['direction'] = 'high'
    with pytest.raises(ShowValidationError, match='illegal rail1 high move from TOP'):
        validate_show(document)


def test_arm_requires_six_positions():
    document = read_demo()
    arm = document['show']['root']['children'][0]['branches'][0]['children'][0]
    arm['args']['positions'] = [0.0] * 5
    with pytest.raises(ShowValidationError, match='exactly 6'):
        validate_show(document)


def test_servo_uart_parameters_must_be_integers():
    document = read_demo()
    servo = (
        document['show']['root']['children'][0]['branches'][3]
        ['children'][0]['branches'][0]
    )
    servo['args']['step_delay_ms'] = 10.5
    with pytest.raises(ShowValidationError, match='must be an integer'):
        validate_show(document)


def test_servo_delay_must_fit_firmware_range():
    document = read_demo()
    servo = (
        document['show']['root']['children'][0]['branches'][3]
        ['children'][0]['branches'][0]
    )
    servo['args']['step_delay_ms'] = 10001
    with pytest.raises(ShowValidationError, match='between 1 and 10000'):
        validate_show(document)
