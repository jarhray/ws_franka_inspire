#!/usr/bin/env python3
"""FR3 executor: mock backend or franky.Robot control with non-blocking motion execution."""

from __future__ import annotations

import math
import queue
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Optional, Tuple, Union

import rclpy
from rclpy.node import Node

from robot_interfaces.msg import ArmAction
from robot_interfaces.srv import HomeArm, ResetArmFault, StopArm
from std_msgs.msg import String

from .franky_backend_impl import (
    BackendResult,
    MockFrankyBackend,
    drain_event_queue,
)


def _finite(x: float) -> bool:
    return math.isfinite(x)


Backend = Union[MockFrankyBackend, Any]


class Fr3FrankyExecutor(Node):
    def __init__(self) -> None:
        super().__init__('fr3_franky_executor')

        self._arm_action_topic = self.declare_parameter(
            'arm_action_topic', '/robot/arm_action'
        ).get_parameter_value().string_value
        self._status_topic = self.declare_parameter(
            'status_topic', '/robot/arm_execution/status'
        ).get_parameter_value().string_value
        self._error_topic = self.declare_parameter(
            'error_topic', '/robot/arm_execution/error'
        ).get_parameter_value().string_value
        self._mock_mode = self.declare_parameter('mock_mode', True).get_parameter_value().bool_value
        self._status_rate_hz = self.declare_parameter(
            'status_rate_hz', 5.0
        ).get_parameter_value().double_value
        self._command_timeout_sec = self.declare_parameter(
            'command_timeout_sec', 2.0
        ).get_parameter_value().double_value
        self._default_reference_frame = self.declare_parameter(
            'default_reference_frame', 'fr3_link0'
        ).get_parameter_value().string_value

        # Franky / FCI (used when mock_mode=false)
        self._fci_hostname = self.declare_parameter(
            'fci_hostname', '172.16.0.2'
        ).get_parameter_value().string_value
        self._franky_controller_mode = self.declare_parameter(
            'franky_controller_mode', 'joint_impedance'
        ).get_parameter_value().string_value
        self._franky_relative_dynamics_factor = self.declare_parameter(
            'franky_relative_dynamics_factor', 1.0
        ).get_parameter_value().double_value
        self._default_velocity_duration_sec = self.declare_parameter(
            'default_velocity_duration_sec', 0.1
        ).get_parameter_value().double_value
        self._velocity_async = self.declare_parameter(
            'velocity_async', True
        ).get_parameter_value().bool_value
        self._home_joint_position = list(
            self.declare_parameter(
                'home_joint_position',
                [0.0, -0.785398, 0.0, -2.35619, 0.0, 1.5708, 0.785398],
            ).get_parameter_value().double_array_value
        )

        self._event_queue: queue.Queue[Tuple[str, str]] = queue.Queue()
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='franky_move')

        self._backend: Backend
        if self._mock_mode:
            self._backend = MockFrankyBackend()
        else:
            from .real_franky_backend import RealFrankyBackend

            self._backend = RealFrankyBackend(
                fci_hostname=self._fci_hostname,
                controller_mode=self._franky_controller_mode,
                relative_dynamics_factor=self._franky_relative_dynamics_factor,
                default_velocity_duration_sec=self._default_velocity_duration_sec,
                home_joint_position=self._home_joint_position,
                velocity_async=self._velocity_async,
            )

        conn = self._backend.connect()

        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._error_pub = self.create_publisher(String, self._error_topic, 10)
        self.create_subscription(ArmAction, self._arm_action_topic, self._on_arm_action, 10)

        self.create_service(StopArm, 'stop_arm', self._on_stop)
        self.create_service(HomeArm, 'home_arm', self._on_home)
        self.create_service(ResetArmFault, 'reset_arm_fault', self._on_reset_fault)

        self._last_command_time = self.get_clock().now()
        self._last_mode = 'none'
        self._last_result = conn.message

        period = 1.0 / max(self._status_rate_hz, 0.1)
        self.create_timer(period, self._on_status_timer)

        self._publish_status(conn.message)
        if not conn.ok:
            self._publish_error(conn.message)

        self.get_logger().info(
            f'fr3_franky_executor: sub={self._arm_action_topic} status={self._status_topic} '
            f'error={self._error_topic} mock_mode={self._mock_mode} fci_hostname={self._fci_hostname!r}'
        )

    def destroy_node(self) -> bool:
        self._executor.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()

    def _publish_status(self, text: str) -> None:
        out = String()
        out.data = text
        self._status_pub.publish(out)

    def _publish_error(self, text: str) -> None:
        out = String()
        out.data = text
        self._error_pub.publish(out)
        self.get_logger().error(text)

    def _drain_worker_events(self) -> None:
        for kind, msg in drain_event_queue(self._event_queue):
            if kind == 'status':
                self._publish_status(msg)
                self._last_result = msg
            elif kind == 'error':
                self._publish_error(msg)
                self._last_result = msg

    def _submit_backend(
        self,
        label: str,
        fn,
        *,
        on_ok_status: Optional[str] = None,
    ) -> None:
        def run() -> None:
            try:
                result: BackendResult = fn()
                if result.ok:
                    self._event_queue.put(('status', f'{label}: {result.message}'))
                else:
                    self._event_queue.put(('error', f'{label} failed: {result.message}'))
            except Exception as e:
                self._event_queue.put(('error', f'{label} exception: {e!s}'))

        self._executor.submit(run)
        if on_ok_status:
            self._publish_status(on_ok_status)

    def _on_arm_action(self, msg: ArmAction) -> None:
        if msg.reference_frame == '':
            msg.reference_frame = self._default_reference_frame

        self._last_command_time = self.get_clock().now()
        self._last_mode = str(msg.control_mode)

        if self._mock_mode:
            result = self._backend.execute_arm_action(msg)
            self._last_result = result.message
            if result.ok:
                self._publish_status(f'execute mode={msg.control_mode}: {result.message}')
            else:
                self._publish_error(f'execute mode={msg.control_mode} failed: {result.message}')
            return

        self._submit_backend(
            f'execute mode={msg.control_mode}',
            lambda: self._backend.execute_arm_action(msg),
            on_ok_status=f'motion queued (mode={msg.control_mode})',
        )

    def _on_stop(self, request: StopArm.Request, response: StopArm.Response) -> StopArm.Response:
        if self._mock_mode:
            result = self._backend.stop(bool(request.immediate))
            response.success = bool(result.ok)
            response.message = result.message
            if result.ok:
                self._publish_status(f'stop_arm: {result.message}')
            else:
                self._publish_error(f'stop_arm failed: {result.message}')
            return response

        self._submit_backend(
            'stop_arm',
            lambda: self._backend.stop(bool(request.immediate)),
            on_ok_status='stop_arm queued',
        )
        response.success = True
        response.message = 'stop_arm submitted to worker'
        return response

    def _on_home(self, request: HomeArm.Request, response: HomeArm.Response) -> HomeArm.Response:
        if self._mock_mode:
            result = self._backend.home(bool(request.wait), float(request.timeout_sec))
            response.success = bool(result.ok)
            response.message = result.message
            if result.ok:
                self._publish_status(f'home_arm: {result.message}')
            else:
                self._publish_error(f'home_arm failed: {result.message}')
            return response

        self._submit_backend(
            'home_arm',
            lambda: self._backend.home(bool(request.wait), float(request.timeout_sec)),
            on_ok_status='home_arm queued',
        )
        response.success = True
        response.message = 'home_arm submitted to worker'
        return response

    def _on_reset_fault(
        self, request: ResetArmFault.Request, response: ResetArmFault.Response
    ) -> ResetArmFault.Response:
        if self._mock_mode:
            result = self._backend.reset_fault(bool(request.hard_reset), float(request.timeout_sec))
            response.success = bool(result.ok)
            response.message = result.message
            if result.ok:
                self._publish_status(f'reset_arm_fault: {result.message}')
            else:
                self._publish_error(f'reset_arm_fault failed: {result.message}')
            return response

        self._submit_backend(
            'reset_arm_fault',
            lambda: self._backend.reset_fault(bool(request.hard_reset), float(request.timeout_sec)),
            on_ok_status='reset_arm_fault queued',
        )
        response.success = True
        response.message = 'reset_arm_fault submitted to worker'
        return response

    def _on_status_timer(self) -> None:
        self._drain_worker_events()

        now = self.get_clock().now()
        age_sec = (now - self._last_command_time).nanoseconds / 1e9
        if _finite(age_sec) and age_sec > self._command_timeout_sec:
            self._publish_error(
                f'command timeout: no ArmAction received for {age_sec:.2f}s '
                f'(limit={self._command_timeout_sec:.2f}s)'
            )
            self._last_command_time = now
            return
        self._publish_status(
            f'heartbeat mode={self._last_mode} last_result="{self._last_result}" age={age_sec:.2f}s'
        )


def main(args: Optional[Tuple[str, ...]] = None) -> None:
    rclpy.init(args=args)
    node = Fr3FrankyExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
