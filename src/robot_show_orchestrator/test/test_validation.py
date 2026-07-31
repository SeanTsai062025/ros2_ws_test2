from copy import deepcopy

import pytest

from robot_show_orchestrator.validation import (
    load_show,
    ShowValidationError,
    validate_show,
)


def action(target='screen', node_id='action'):
    node = {
        'type': 'action',
        'target': target,
        'timeout_s': 1.0,
    }
    if node_id is not None:
        node['id'] = node_id
    if target == 'screen':
        node.update({
            'command': 'face',
            'args': {'face_type': 'HAPPY'},
        })
    elif target == 'arm':
        node.update({
            'command': 'joint_move',
            'args': {
                'positions': [0.0] * 6,
                'speed_scaling': 0.2,
            },
        })
    elif target in {'rail1', 'rail2'}:
        node.update({
            'command': 'move',
            'args': {'steps': 16000, 'direction': 'low'},
        })
    else:
        node.update({
            'command': 'move_to',
            'args': {'target_degree': 90, 'step_delay_ms': 10},
        })
    return node


def show(root, requirements=None):
    return {
        'schema_version': '1.0',
        'show': {
            'id': 'test',
            'requirements': requirements or {},
            'defaults': {
                'on_failure': 'abort_show',
                'command_timeout_s': 1.0,
            },
            'root': root,
        },
    }


def test_node_ids_may_be_omitted_or_repeated():
    document = show({
        'type': 'sequence',
        'children': [
            action(node_id='same_name'),
            action(node_id='same_name'),
            action(node_id=None),
        ],
    })
    validate_show(document)


def test_provided_node_id_must_be_a_non_empty_string():
    document = show(action(node_id=''))
    with pytest.raises(ShowValidationError, match='when provided'):
        validate_show(document)


def test_load_show_remembers_file_and_line_for_validation_errors(tmp_path):
    show_file = tmp_path / 'bad_show.yaml'
    show_file.write_text(
        'schema_version: "1.0"\n'
        'show:\n'
        '  id: test\n'
        '  root:\n'
        '    type: action\n'
        '    target: arm\n'
        '    command: joint_move\n'
        '    args:\n'
        '      positions: [0, 0]\n'
        '      speed_scaling: 0.5\n',
        encoding='utf-8',
    )
    with pytest.raises(ShowValidationError) as caught:
        load_show(str(show_file))
    assert f'{show_file}:5' in str(caught.value)
    assert 'positions must contain exactly 6 values' in str(caught.value)


def test_parallel_same_device_is_rejected():
    document = show({
        'type': 'parallel',
        'branches': [
            action(node_id='first'),
            action(node_id='second'),
        ],
    })
    with pytest.raises(ShowValidationError, match='parallel resource conflict'):
        validate_show(document)


def test_illegal_rail_direction_is_rejected():
    rail = action('rail1')
    rail['args']['direction'] = 'high'
    document = show(rail, {'rail1_initial_state': 'TOP'})
    with pytest.raises(ShowValidationError, match='illegal rail1 high move from TOP'):
        validate_show(document)


def test_arm_requires_six_positions():
    arm = action('arm')
    arm['args']['positions'] = [0.0] * 5
    with pytest.raises(ShowValidationError, match='exactly 6'):
        validate_show(show(arm))


def test_servo_uart_parameters_must_be_integers():
    servo = action('servo1')
    servo['args']['step_delay_ms'] = 10.5
    with pytest.raises(ShowValidationError, match='must be an integer'):
        validate_show(show(servo))


def test_servo_delay_must_fit_firmware_range():
    servo = deepcopy(action('servo1'))
    servo['args']['step_delay_ms'] = 10001
    with pytest.raises(ShowValidationError, match='between 1 and 10000'):
        validate_show(show(servo))
