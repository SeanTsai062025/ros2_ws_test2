"""ROS 2 node that adapts validated show workflows to existing robot topics."""

from __future__ import annotations

import asyncio
from pathlib import Path
import threading
from typing import Dict, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from dexter_interfaces.msg import (
    CommandResult,
    DeviceCommand,
    JointCommand,
    ShowState,
)
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .runtime import ActionRequest, ActionResult, RuntimeSnapshot, WorkflowRuntime
from .validation import load_show, show_resources, ShowValidationError


class RobotShowOrchestrator(Node):
    """Persistent high-level show controller."""

    def __init__(self) -> None:
        super().__init__('show_orchestrator')
        default_show = str(
            Path(get_package_share_directory('robot_show_orchestrator'))
            / 'config'
            / 'demo_show.yaml'
        )
        self.declare_parameter('show_file', default_show)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('require_readiness', True)
        self.declare_parameter('safety_confirmed', False)

        reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.joint_pub = self.create_publisher(
            JointCommand, '/joint_command', reliable
        )
        self.device_pub = self.create_publisher(
            DeviceCommand, '/device_command', reliable
        )
        self.device_cancel_pub = self.create_publisher(
            DeviceCommand, '/device_cancel', reliable
        )
        self.arm_cancel_pub = self.create_publisher(
            DeviceCommand, '/arm_cancel', reliable
        )
        self.state_pub = self.create_publisher(
            ShowState, '~/state', state_qos
        )

        self.arm_result_sub = self.create_subscription(
            CommandResult,
            '/arm_command_result',
            self._on_result,
            reliable,
        )
        self.device_result_sub = self.create_subscription(
            CommandResult,
            '/device_command_result',
            self._on_result,
            reliable,
        )
        self.commander_ready_sub = self.create_subscription(
            Bool, '/commander/ready', self._on_commander_ready, state_qos
        )
        self.esp32_ready_sub = self.create_subscription(
            Bool, '/esp32/connected', self._on_esp32_ready, state_qos
        )

        self.start_service = self.create_service(
            Trigger, '~/start', self._start_callback
        )
        self.pause_service = self.create_service(
            Trigger, '~/pause', self._pause_callback
        )
        self.resume_service = self.create_service(
            Trigger, '~/resume', self._resume_callback
        )
        self.stop_service = self.create_service(
            Trigger, '~/stop', self._stop_callback
        )
        self.reload_service = self.create_service(
            Trigger, '~/reload', self._reload_callback
        )

        self._document = None
        self._runtime: Optional[WorkflowRuntime] = None
        self._runtime_thread: Optional[threading.Thread] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._pending: Dict[
            Tuple[str, str], Tuple[asyncio.AbstractEventLoop, asyncio.Future]
        ] = {}
        self._pending_lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._commander_ready = False
        self._esp32_ready = False
        self._load_current_show()
        self._publish_idle_state('show loaded')

        if bool(self.get_parameter('auto_start').value):
            self._auto_start_timer = self.create_timer(0.5, self._auto_start_once)
        else:
            self._auto_start_timer = None

    def _load_current_show(self) -> None:
        show_file = str(self.get_parameter('show_file').value)
        self._document = load_show(show_file)
        self.get_logger().info(
            f'Loaded show {self._document["show"]["id"]} from {show_file}'
        )

    def _on_commander_ready(self, msg: Bool) -> None:
        self._commander_ready = bool(msg.data)

    def _on_esp32_ready(self, msg: Bool) -> None:
        self._esp32_ready = bool(msg.data)

    def _readiness_error(self) -> Optional[str]:
        if not bool(self.get_parameter('require_readiness').value):
            return None
        if not bool(self.get_parameter('safety_confirmed').value):
            return (
                'safety_confirmed is false; verify E-stop and hardware safety, '
                'then set the parameter true'
            )
        resources = show_resources(self._document)
        if 'arm' in resources and not self._commander_ready:
            return 'Commander is not ready'
        if resources - {'arm'} and not self._esp32_ready:
            return 'ESP32 UART bridge is not connected'
        return None

    def _start_show(self) -> Tuple[bool, str]:
        with self._lifecycle_lock:
            if self._runtime_thread is not None and self._runtime_thread.is_alive():
                return False, 'a show is already active'
            try:
                self._load_current_show()
            except ShowValidationError as error:
                return False, f'show validation failed: {error}'
            readiness_error = self._readiness_error()
            if readiness_error:
                return False, readiness_error

            self._runtime = WorkflowRuntime(
                self._document,
                dispatch=self._dispatch,
                cancel=self._cancel,
                state_callback=self._publish_snapshot,
            )
            run_id = self._runtime.show_run_id
            self._runtime_thread = threading.Thread(
                target=self._run_runtime,
                name=f'show-{run_id}',
                daemon=True,
            )
            self._runtime_thread.start()
            return True, f'started show_run_id={run_id}'

    def _run_runtime(self) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop
        try:
            loop.run_until_complete(self._runtime.run())
        finally:
            with self._pending_lock:
                pending = list(self._pending.values())
                self._pending.clear()
            for pending_loop, future in pending:
                if pending_loop is loop and not future.done():
                    future.cancel()
            self._loop = None
            loop.close()

    async def _dispatch(self, request: ActionRequest) -> ActionResult:
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        key = (request.show_run_id, request.command_id)
        with self._pending_lock:
            self._pending[key] = (loop, future)
        try:
            if request.target == 'arm':
                message = JointCommand()
                self._fill_metadata(message, request)
                message.positions = [
                    float(value) for value in request.args['positions']
                ]
                message.speed_scaling = float(
                    request.args['speed_scaling']
                )
                self.joint_pub.publish(message)
            else:
                message = self._device_message(request)
                self.device_pub.publish(message)
            return await future
        finally:
            with self._pending_lock:
                self._pending.pop(key, None)

    @staticmethod
    def _fill_metadata(message, request: ActionRequest) -> None:
        message.show_run_id = request.show_run_id
        message.node_id = request.node_id
        message.command_id = request.command_id
        message.attempt = request.attempt

    def _device_message(self, request: ActionRequest) -> DeviceCommand:
        message = DeviceCommand()
        self._fill_metadata(message, request)
        message.target = request.target
        if request.target in {'rail1', 'rail2'}:
            direction = str(request.args['direction']).lower()
            message.command = (
                f'{request.target} {int(request.args["steps"])} '
                f'{direction}'
            )
        elif request.target in {'servo1', 'servo2'}:
            message.command = (
                f'{request.target} {int(request.args["target_degree"])} '
                f'{int(request.args["step_delay_ms"])}'
            )
        elif request.target == 'screen':
            message.command = f'FACE {str(request.args["face_type"]).upper()}'
        else:  # Validation prevents this path.
            raise RuntimeError(f'unsupported target: {request.target}')
        return message

    def _cancel(self, request: ActionRequest) -> None:
        message = DeviceCommand()
        self._fill_metadata(message, request)
        message.target = request.target
        if request.target == 'arm':
            self.arm_cancel_pub.publish(message)
        else:
            self.device_cancel_pub.publish(message)

    def _on_result(self, msg: CommandResult) -> None:
        key = (msg.show_run_id, msg.command_id)
        with self._pending_lock:
            pending = self._pending.get(key)
        if pending is None:
            self.get_logger().warning(
                f'Ignoring stale result: run={msg.show_run_id} '
                f'command={msg.command_id}'
            )
            return
        loop, future = pending

        def set_result() -> None:
            if not future.done():
                future.set_result(ActionResult(msg.status, msg.detail))

        loop.call_soon_threadsafe(set_result)

    def _publish_snapshot(self, snapshot: RuntimeSnapshot) -> None:
        message = ShowState()
        message.show_run_id = snapshot.show_run_id
        message.show_id = snapshot.show_id
        message.status = snapshot.status
        message.active_node_ids = list(snapshot.active_node_ids)
        message.device_names = sorted(snapshot.device_states)
        message.device_states = [
            snapshot.device_states[name] for name in message.device_names
        ]
        message.detail = snapshot.detail
        self.state_pub.publish(message)

    def _publish_idle_state(self, detail: str) -> None:
        message = ShowState()
        message.show_run_id = ''
        message.show_id = (
            self._document['show']['id'] if self._document is not None else ''
        )
        message.status = 'IDLE'
        message.detail = detail
        self.state_pub.publish(message)

    def _auto_start_once(self) -> None:
        self._auto_start_timer.cancel()
        accepted, message = self._start_show()
        if accepted:
            self.get_logger().info(message)
        else:
            self.get_logger().error(f'Auto-start rejected: {message}')

    def _start_callback(self, request, response):
        del request
        response.success, response.message = self._start_show()
        return response

    def _pause_callback(self, request, response):
        del request
        runtime = self._runtime
        response.success = runtime is not None and runtime.pause()
        response.message = 'soft pause requested' if response.success else 'no running show'
        return response

    def _resume_callback(self, request, response):
        del request
        runtime = self._runtime
        response.success = runtime is not None and runtime.resume()
        response.message = 'show resumed' if response.success else 'show is not paused'
        return response

    def _stop_callback(self, request, response):
        del request
        runtime = self._runtime
        response.success = runtime is not None and runtime.stop()
        response.message = 'stop requested' if response.success else 'no running show'
        return response

    def _reload_callback(self, request, response):
        del request
        if self._runtime_thread is not None and self._runtime_thread.is_alive():
            response.success = False
            response.message = 'cannot reload while a show is active'
            return response
        try:
            self._load_current_show()
        except ShowValidationError as error:
            response.success = False
            response.message = str(error)
            return response
        self._publish_idle_state('show reloaded')
        response.success = True
        response.message = f'loaded show {self._document["show"]["id"]}'
        return response

    def destroy_node(self) -> None:
        if self._runtime is not None:
            self._runtime.stop()
        if self._runtime_thread is not None:
            self._runtime_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotShowOrchestrator()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
