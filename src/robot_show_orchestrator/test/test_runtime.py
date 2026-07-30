import asyncio
import time

from robot_show_orchestrator.runtime import ActionResult, WorkflowRuntime
from robot_show_orchestrator.validation import validate_show


def action(node_id, target):
    if target == 'screen':
        return {
            'id': node_id,
            'type': 'action',
            'target': 'screen',
            'command': 'face',
            'args': {'face_type': 'HAPPY'},
            'timeout_s': 1.0,
        }
    return {
        'id': node_id,
        'type': 'action',
        'target': 'servo1',
        'command': 'move_to',
        'args': {'target_degree': 90, 'step_delay_ms': 10},
        'timeout_s': 1.0,
    }


def show(root):
    return {
        'schema_version': '1.0',
        'show': {
            'id': 'test',
            'requirements': {},
            'defaults': {
                'on_failure': 'abort_show',
                'command_timeout_s': 1.0,
            },
            'root': root,
        },
    }


def test_sequence_waits_for_each_result():
    document = show({
        'id': 'root',
        'type': 'sequence',
        'children': [action('first', 'screen'), action('second', 'screen')],
    })
    validate_show(document)
    events = []

    async def dispatch(request):
        events.append(('start', request.node_id))
        await asyncio.sleep(0.01)
        events.append(('done', request.node_id))
        return ActionResult('SUCCEEDED', 'ok')

    runtime = WorkflowRuntime(document, dispatch, lambda request: None, lambda state: None)
    assert asyncio.run(runtime.run()) == 'SUCCEEDED'
    assert events == [
        ('start', 'first'),
        ('done', 'first'),
        ('start', 'second'),
        ('done', 'second'),
    ]


def test_parallel_branches_run_concurrently_and_join():
    document = show({
        'id': 'root',
        'type': 'sequence',
        'children': [
            {
                'id': 'parallel',
                'type': 'parallel',
                'join': 'all',
                'branches': [
                    action('screen', 'screen'),
                    action('servo', 'servo1'),
                ],
            },
            {'id': 'after', 'type': 'delay', 'duration_s': 0.001},
        ],
    })
    validate_show(document)
    starts = {}
    finishes = {}
    snapshots = []

    async def dispatch(request):
        starts[request.node_id] = time.monotonic()
        await asyncio.sleep(0.03)
        finishes[request.node_id] = time.monotonic()
        return ActionResult('SUCCEEDED', 'ok')

    runtime = WorkflowRuntime(
        document,
        dispatch,
        lambda request: None,
        lambda snapshot: snapshots.append((time.monotonic(), snapshot)),
    )
    assert asyncio.run(runtime.run()) == 'SUCCEEDED'
    assert abs(starts['screen'] - starts['servo']) < 0.02
    after_time, _ = next(
        item for item in snapshots if 'after' in item[1].active_node_ids
    )
    assert finishes['screen'] <= after_time
    assert finishes['servo'] <= after_time


def test_timeout_cancels_and_marks_device_unknown():
    document = show(action('slow', 'servo1'))
    validate_show(document)
    document['show']['root']['timeout_s'] = 0.01
    cancelled = []

    async def dispatch(request):
        await asyncio.sleep(1.0)
        return ActionResult('SUCCEEDED', 'late')

    runtime = WorkflowRuntime(
        document, dispatch, cancelled.append, lambda state: None
    )
    assert asyncio.run(runtime.run()) == 'FAILED'
    assert len(cancelled) == 1
    assert runtime.device_states['servo1'] == 'UNKNOWN'


def test_soft_pause_allows_active_action_to_finish_but_blocks_next():
    document = show({
        'id': 'root',
        'type': 'sequence',
        'children': [action('first', 'screen'), action('second', 'screen')],
    })
    validate_show(document)
    started = []
    first_started = asyncio.Event()
    release_first = asyncio.Event()

    async def dispatch(request):
        started.append(request.node_id)
        if request.node_id == 'first':
            first_started.set()
            await release_first.wait()
        return ActionResult('SUCCEEDED', 'ok')

    runtime = WorkflowRuntime(document, dispatch, lambda request: None, lambda state: None)

    async def scenario():
        task = asyncio.create_task(runtime.run())
        await first_started.wait()
        assert runtime.pause()
        release_first.set()
        await asyncio.sleep(0.05)
        assert started == ['first']
        assert runtime.resume()
        assert await task == 'SUCCEEDED'

    asyncio.run(scenario())
    assert started == ['first', 'second']
