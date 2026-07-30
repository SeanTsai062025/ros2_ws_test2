"""Async workflow runtime independent of ROS transport details."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
import threading
from typing import Any, Awaitable, Callable, Dict, Mapping
import uuid


@dataclass(frozen=True)
class ActionRequest:
    """One concrete attempt generated from an action node."""

    show_run_id: str
    node_id: str
    command_id: str
    attempt: int
    target: str
    command: str
    args: Mapping[str, Any]
    timeout_s: float


@dataclass(frozen=True)
class ActionResult:
    """Transport-independent terminal command result."""

    status: str
    detail: str


@dataclass(frozen=True)
class RuntimeSnapshot:
    """State published whenever execution changes materially."""

    show_run_id: str
    show_id: str
    status: str
    active_node_ids: tuple[str, ...]
    device_states: Dict[str, str]
    detail: str


class WorkflowError(RuntimeError):
    """A workflow action failed or the show was stopped."""


Dispatch = Callable[[ActionRequest], Awaitable[ActionResult]]
Cancel = Callable[[ActionRequest], None]
StateCallback = Callable[[RuntimeSnapshot], None]


class WorkflowRuntime:
    """Execute action, sequence, parallel, and delay nodes."""

    def __init__(
        self,
        document: Mapping[str, Any],
        dispatch: Dispatch,
        cancel: Cancel,
        state_callback: StateCallback,
    ) -> None:
        self.document = document
        self.dispatch = dispatch
        self.cancel_callback = cancel
        self.state_callback = state_callback
        self.show_run_id = str(uuid.uuid4())
        self.show = document['show']
        self.defaults = self.show.get('defaults', {})

        requirements = self.show.get('requirements', {})
        self.device_states: Dict[str, str] = {
            'arm': 'READY',
            'rail1': str(requirements.get('rail1_initial_state', 'UNKNOWN')).upper(),
            'rail2': str(requirements.get('rail2_initial_state', 'UNKNOWN')).upper(),
            'servo1': 'READY',
            'servo2': 'READY',
            'screen': 'READY',
        }
        self.status = 'IDLE'
        self.detail = ''
        self._active_nodes: set[str] = set()
        self._active_requests: Dict[str, ActionRequest] = {}
        self._state_lock = threading.Lock()
        self._paused = threading.Event()
        self._stopped = threading.Event()

    def pause(self) -> bool:
        """Soft-pause before the next not-yet-started node."""
        if self.status not in {'RUNNING', 'PAUSED'} or self._stopped.is_set():
            return False
        self._paused.set()
        self.status = 'PAUSED'
        self.detail = 'soft pause requested; in-flight nodes continue'
        self._publish_state()
        return True

    def resume(self) -> bool:
        if not self._paused.is_set() or self._stopped.is_set():
            return False
        self._paused.clear()
        self.status = 'RUNNING'
        self.detail = 'resumed'
        self._publish_state()
        return True

    def stop(self) -> bool:
        """Stop scheduling and request cancellation of every active action."""
        if self.status not in {'RUNNING', 'PAUSED'}:
            return False
        self._stopped.set()
        self._paused.clear()
        self._cancel_active()
        self.detail = 'stop requested'
        self._publish_state()
        return True

    async def run(self) -> str:
        self.status = 'RUNNING'
        self.detail = 'show started'
        self._publish_state()
        try:
            await self._execute_node(self.show['root'])
            if self._stopped.is_set():
                raise WorkflowError('show stopped')
        except WorkflowError as error:
            if self._stopped.is_set():
                self.status = 'STOPPED'
            else:
                self.status = 'FAILED'
                self._cancel_active()
            self.detail = str(error)
        except asyncio.CancelledError:
            self._stopped.set()
            self._cancel_active()
            self.status = 'STOPPED'
            self.detail = 'show task cancelled'
        except Exception as error:  # Keep the node alive after malformed adapters.
            self.status = 'FAILED'
            self._cancel_active()
            self.detail = f'unexpected runtime error: {error}'
        else:
            self.status = 'SUCCEEDED'
            self.detail = 'show completed'
        finally:
            self._active_nodes.clear()
            self._publish_state()
        return self.status

    async def _wait_before_node(self) -> None:
        while self._paused.is_set() and not self._stopped.is_set():
            await asyncio.sleep(0.02)
        if self._stopped.is_set():
            raise WorkflowError('show stopped')

    async def _execute_node(self, node: Mapping[str, Any]) -> None:
        await self._wait_before_node()
        node_type = node['type']
        if node_type == 'action':
            await self._execute_action(node)
        elif node_type == 'delay':
            await self._execute_delay(node)
        elif node_type == 'sequence':
            for child in node['children']:
                await self._execute_node(child)
        elif node_type == 'parallel':
            await self._execute_parallel(node)
        else:
            raise WorkflowError(f'unsupported node type: {node_type}')

    async def _execute_parallel(self, node: Mapping[str, Any]) -> None:
        tasks = [
            asyncio.create_task(self._execute_node(branch))
            for branch in node['branches']
        ]
        done, pending = await asyncio.wait(
            tasks, return_when=asyncio.FIRST_EXCEPTION
        )
        failure = next(
            (
                task.exception()
                for task in done
                if not task.cancelled() and task.exception() is not None
            ),
            None,
        )
        if failure is not None:
            self._cancel_active()
            for task in pending:
                task.cancel()
            await asyncio.gather(*pending, return_exceptions=True)
            if isinstance(failure, WorkflowError):
                raise failure
            raise WorkflowError(str(failure))
        if pending:
            await asyncio.gather(*pending)

    async def _execute_delay(self, node: Mapping[str, Any]) -> None:
        node_id = node['id']
        self._active_nodes.add(node_id)
        self.detail = f'delay {node_id}'
        self._publish_state()
        try:
            remaining = float(node['duration_s'])
            while remaining > 0:
                if self._stopped.is_set():
                    raise WorkflowError('show stopped')
                interval = min(remaining, 0.05)
                await asyncio.sleep(interval)
                remaining -= interval
        finally:
            self._active_nodes.discard(node_id)
            self._publish_state()

    async def _execute_action(self, node: Mapping[str, Any]) -> None:
        request = ActionRequest(
            show_run_id=self.show_run_id,
            node_id=node['id'],
            command_id=str(uuid.uuid4()),
            attempt=1,
            target=node['target'],
            command=node['command'],
            args=node.get('args', {}),
            timeout_s=float(
                node.get(
                    'timeout_s',
                    self.defaults.get('command_timeout_s', 30.0),
                )
            ),
        )
        self._active_nodes.add(request.node_id)
        with self._state_lock:
            self._active_requests[request.command_id] = request
        self.device_states[request.target] = self._busy_state(request)
        self.detail = (
            f'executing node={request.node_id} command={request.command_id}'
        )
        self._publish_state()

        try:
            result = await asyncio.wait_for(
                self.dispatch(request), timeout=request.timeout_s
            )
        except asyncio.TimeoutError as error:
            self.cancel_callback(request)
            self.device_states[request.target] = 'UNKNOWN'
            raise WorkflowError(
                f'node {request.node_id} timed out after {request.timeout_s:.3f}s'
            ) from error
        except asyncio.CancelledError:
            self.cancel_callback(request)
            self.device_states[request.target] = 'UNKNOWN'
            raise
        finally:
            with self._state_lock:
                self._active_requests.pop(request.command_id, None)
            self._active_nodes.discard(request.node_id)
            self._publish_state()

        if result.status != 'SUCCEEDED':
            if result.status != 'REJECTED':
                self.device_states[request.target] = 'UNKNOWN'
            raise WorkflowError(
                f'node {request.node_id} {result.status}: {result.detail}'
            )
        self.device_states[request.target] = self._success_state(request)
        self.detail = f'node {request.node_id} succeeded: {result.detail}'
        self._publish_state()

    def _cancel_active(self) -> None:
        with self._state_lock:
            requests = list(self._active_requests.values())
        for request in requests:
            self.cancel_callback(request)
            self.device_states[request.target] = 'UNKNOWN'

    @staticmethod
    def _busy_state(request: ActionRequest) -> str:
        if request.target in {'rail1', 'rail2'}:
            direction = str(request.args['direction']).upper()
            return f'MOVING_{direction}'
        return 'BUSY'

    @staticmethod
    def _success_state(request: ActionRequest) -> str:
        if request.target in {'rail1', 'rail2'}:
            direction = str(request.args['direction']).lower()
            return 'BOTTOM' if direction == 'low' else 'TOP'
        if request.target in {'servo1', 'servo2'}:
            return f'AT_{request.args["target_degree"]}_UNVERIFIED'
        if request.target == 'screen':
            face = str(request.args['face_type']).upper().replace(' ', '_')
            return f'FACE_{face}'
        return 'READY'

    def _publish_state(self) -> None:
        snapshot = RuntimeSnapshot(
            show_run_id=self.show_run_id,
            show_id=self.show['id'],
            status=self.status,
            active_node_ids=tuple(sorted(self._active_nodes)),
            device_states=dict(self.device_states),
            detail=self.detail,
        )
        self.state_callback(snapshot)
