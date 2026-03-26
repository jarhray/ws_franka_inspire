#!/usr/bin/env python3
"""FR3 executor framework with a mockable backend."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from robot_interfaces.msg import ArmAction
from robot_interfaces.srv import HomeArm, ResetArmFault, StopArm
from std_msgs.msg import String


ARM_JOINT_POSITION = 0
ARM_JOINT_VELOCITY = 1
ARM_CARTESIAN_POSE = 2
ARM_CARTESIAN_VELOCITY = 3


def _finite(x: float) -> bool:
    return math.isfinite(x)


def _fmt_vec(v: List[float], max_items: int = 3) -> str:
    if not v:
        return '[]'
    clipped = v[:max_items]
    suffix = '...' if len(v) > max_items else ''
    return '[' + ', '.join(f'{x:.4f}' for x in clipped) + suffix + ']'


@dataclass
class BackendResult:
    ok: bool
    message: str


class FrankyBackend:
    """Mockable backend interface for FR3 motion execution."""

    def __init__(self, mock_mode: bool) -> None:
        self._mock_mode = mock_mode
        self._connected = False
        self._fault_active = False

    def connect(self) -> BackendResult:
        self._connected = True
        mode = 'mock' if self._mock_mode else 'real-placeholder'
        return BackendResult(True, f'backend connected ({mode})')

    def execute_arm_action(self, msg: ArmAction) -> BackendResult:
        if not self._connected:
            return BackendResult(False, 'backend not connected')
        if self._fault_active:
            return BackendResult(False, 'backend fault active; call /reset_arm_fault')
        if msg.control_mode == ARM_JOINT_POSITION:
            return BackendResult(True, f'joint_position target={_fmt_vec(list(msg.joint_position))}')
        if msg.control_mode == ARM_JOINT_VELOCITY:
            return BackendResult(True, f'joint_velocity target={_fmt_vec(list(msg.joint_velocity))}')
        if msg.control_mode == ARM_CARTESIAN_POSE:
            p = msg.cartesian_pose.position
            return BackendResult(True, f'cartesian_pose target=({p.x:.4f}, {p.y:.4f}, {p.z:.4f})')
        if msg.control_mode == ARM_CARTESIAN_VELOCITY:
            lv = msg.cartesian_velocity.linear
            av = msg.cartesian_velocity.angular
            return BackendResult(
                True,
                (
                    'cartesian_velocity '
                    f'lin=({lv.x:.4f}, {lv.y:.4f}, {lv.z:.4f}) '
                    f'ang=({av.x:.4f}, {av.y:.4f}, {av.z:.4f})'
                ),
            )
        return BackendResult(False, f'unsupported control_mode={msg.control_mode}')

    def stop(self, immediate: bool) -> BackendResult:
        if not self._connected:
            return BackendResult(False, 'backend not connected')
        return BackendResult(True, f'stop accepted (immediate={immediate})')

    def home(self, wait: bool, timeout_sec: float) -> BackendResult:
        if not self._connected:
            return BackendResult(False, 'backend not connected')
        return BackendResult(True, f'home accepted (wait={wait}, timeout={timeout_sec:.2f}s)')

    def reset_fault(self, hard_reset: bool, timeout_sec: float) -> BackendResult:
        if not self._connected:
            return BackendResult(False, 'backend not connected')
        self._fault_active = False
        return BackendResult(
            True, f'reset_fault accepted (hard_reset={hard_reset}, timeout={timeout_sec:.2f}s)'
        )


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

        self._backend = FrankyBackend(mock_mode=self._mock_mode)
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
        self.get_logger().info(
            f'fr3_franky_executor: sub={self._arm_action_topic} status={self._status_topic} '
            f'error={self._error_topic} mock_mode={self._mock_mode}'
        )

    def _publish_status(self, text: str) -> None:
        out = String()
        out.data = text
        self._status_pub.publish(out)

    def _publish_error(self, text: str) -> None:
        out = String()
        out.data = text
        self._error_pub.publish(out)
        self.get_logger().error(text)

    def _on_arm_action(self, msg: ArmAction) -> None:
        if msg.reference_frame == '':
            msg.reference_frame = self._default_reference_frame

        result = self._backend.execute_arm_action(msg)
        self._last_command_time = self.get_clock().now()
        self._last_mode = str(msg.control_mode)
        self._last_result = result.message

        if result.ok:
            self._publish_status(f'execute mode={msg.control_mode}: {result.message}')
        else:
            self._publish_error(f'execute mode={msg.control_mode} failed: {result.message}')

    def _on_stop(self, request: StopArm.Request, response: StopArm.Response) -> StopArm.Response:
        result = self._backend.stop(bool(request.immediate))
        response.success = bool(result.ok)
        response.message = result.message
        if result.ok:
            self._publish_status(f'stop_arm: {result.message}')
        else:
            self._publish_error(f'stop_arm failed: {result.message}')
        return response

    def _on_home(self, request: HomeArm.Request, response: HomeArm.Response) -> HomeArm.Response:
        result = self._backend.home(bool(request.wait), float(request.timeout_sec))
        response.success = bool(result.ok)
        response.message = result.message
        if result.ok:
            self._publish_status(f'home_arm: {result.message}')
        else:
            self._publish_error(f'home_arm failed: {result.message}')
        return response

    def _on_reset_fault(
        self, request: ResetArmFault.Request, response: ResetArmFault.Response
    ) -> ResetArmFault.Response:
        result = self._backend.reset_fault(bool(request.hard_reset), float(request.timeout_sec))
        response.success = bool(result.ok)
        response.message = result.message
        if result.ok:
            self._publish_status(f'reset_arm_fault: {result.message}')
        else:
            self._publish_error(f'reset_arm_fault failed: {result.message}')
        return response

    def _on_status_timer(self) -> None:
        now = self.get_clock().now()
        age_sec = (now - self._last_command_time).nanoseconds / 1e9
        if _finite(age_sec) and age_sec > self._command_timeout_sec:
            self._publish_error(
                f'command timeout: no ArmAction received for {age_sec:.2f}s '
                f'(limit={self._command_timeout_sec:.2f}s)'
            )
            # Avoid flooding error topic every timer tick.
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
