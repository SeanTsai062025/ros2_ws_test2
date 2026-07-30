from dexter_interfaces.msg import DeviceCommand
from esp32_motor_controller.uart_bridge_node import (
    encode_cancel,
    encode_command,
    parse_result,
)
import pytest


def command():
    message = DeviceCommand()
    message.show_run_id = 'run-1'
    message.node_id = 'rail_down'
    message.command_id = 'command-1'
    message.attempt = 1
    message.target = 'rail1'
    message.command = 'rail1 16000 low'
    return message


def test_structured_protocol_round_trip_fields():
    message = command()
    assert encode_command(message) == (
        'CMD|run-1|command-1|rail1|rail1 16000 low'
    )
    assert encode_cancel(message) == 'CANCEL|run-1|command-1|rail1'
    assert parse_result(
        'RESULT|run-1|command-1|rail1|SUCCEEDED|OK'
    ) == ('run-1', 'command-1', 'rail1', 'SUCCEEDED', 'OK')


def test_protocol_rejects_delimiter_in_ids():
    message = command()
    message.command_id = 'bad|id'
    with pytest.raises(ValueError, match='cannot contain'):
        encode_command(message)


def test_plain_legacy_reply_is_not_structured():
    assert parse_result('OK: rail1 completed 16000 steps') is None
