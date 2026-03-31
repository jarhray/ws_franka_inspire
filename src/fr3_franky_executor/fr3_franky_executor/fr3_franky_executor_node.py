#!/usr/bin/env python3
"""FR3 executor: mock backend or franky.Robot control with non-blocking motion execution."""

from __future__ import annotations

import math
import queue
import threading
import time
from typing import Any, Optional, Tuple, Union

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

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
        self._command_timeout_warn_interval_sec = self.declare_parameter(
            'command_timeout_warn_interval_sec', 15.0
        ).get_parameter_value().double_value
        self._default_reference_frame = self.declare_parameter(
            'default_reference_frame', 'fr3_link0'
        ).get_parameter_value().string_value
        self._joint_names = list(
            self.declare_parameter(
                'joint_names',
                ['fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7'],
            ).get_parameter_value().string_array_value
        )
        self._state_joint_topic = self.declare_parameter(
            'state_joint_topic', '/franka_robot_state_broadcaster/measured_joint_states'
        ).get_parameter_value().string_value
        self._state_pose_topic = self.declare_parameter(
            'state_pose_topic', '/franka_robot_state_broadcaster/current_pose'
        ).get_parameter_value().string_value
        self._state_twist_topic = self.declare_parameter(
            'state_twist_topic', '/franka_robot_state_broadcaster/desired_end_effector_twist'
        ).get_parameter_value().string_value
        self._state_publish_rate_hz = self.declare_parameter(
            'state_publish_rate_hz', 30.0
        ).get_parameter_value().double_value
        self._state_retry_initial_sec = self.declare_parameter(
            'state_retry_initial_sec', 0.5
        ).get_parameter_value().double_value
        self._state_retry_max_sec = self.declare_parameter(
            'state_retry_max_sec', 8.0
        ).get_parameter_value().double_value

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
        self._position_async = self.declare_parameter(
            'position_async', True
        ).get_parameter_value().bool_value
        self._home_joint_position = list(
            self.declare_parameter(
                'home_joint_position',
                [0.0, -0.785398, 0.0, -2.35619, 0.0, 1.5708, 0.785398],
            ).get_parameter_value().double_array_value
        )

        self._event_queue: queue.Queue[Tuple[str, str]] = queue.Queue()
        self._command_queue: queue.Queue[Tuple[str, tuple[Any, ...]]] = queue.Queue()
        self._state_lock = threading.Lock()
        self._latest_state: Optional[Any] = None
        self._worker_stop_evt = threading.Event()
        self._worker_thread: Optional[threading.Thread] = None

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
                position_async=self._position_async,
            )

        conn = self._backend.connect()

        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._error_pub = self.create_publisher(String, self._error_topic, 10)
        self._joint_pub = self.create_publisher(JointState, self._state_joint_topic, 10)
        self._pose_pub = self.create_publisher(PoseStamped, self._state_pose_topic, 10)
        self._twist_pub = self.create_publisher(TwistStamped, self._state_twist_topic, 10)
        self.create_subscription(ArmAction, self._arm_action_topic, self._on_arm_action, 10)

        self.create_service(StopArm, 'stop_arm', self._on_stop)
        self.create_service(HomeArm, 'home_arm', self._on_home)
        self.create_service(ResetArmFault, 'reset_arm_fault', self._on_reset_fault)

        self._last_command_time = self.get_clock().now()
        self._last_mode = 'none'
        self._last_result = conn.message
        self._state_publish_enabled = (not self._mock_mode) and bool(conn.ok)
        self._state_next_retry_time_sec = 0.0
        self._state_retry_delay_sec = max(0.1, float(self._state_retry_initial_sec))
        self._state_poll_period_sec = 1.0 / max(self._state_publish_rate_hz, 0.1)
        self._motion_in_progress = False
        self._last_timeout_warn_time = None

        period = 1.0 / max(self._status_rate_hz, 0.1)
        self.create_timer(period, self._on_status_timer)
        self.create_timer(self._state_poll_period_sec, self._on_state_timer)

        if not self._mock_mode:
            self._worker_thread = threading.Thread(
                target=self._worker_loop, name='franky_worker', daemon=True
            )
            self._worker_thread.start()

        self._publish_status(conn.message)
        if not conn.ok:
            self._publish_error(conn.message)

        self.get_logger().info(
            f'fr3_franky_executor: sub={self._arm_action_topic} status={self._status_topic} '
            f'error={self._error_topic} state_joint={self._state_joint_topic} '
            f'state_pose={self._state_pose_topic} state_twist={self._state_twist_topic} '
            f'mock_mode={self._mock_mode} fci_hostname={self._fci_hostname!r}'
        )

    def destroy_node(self) -> bool:
        self._worker_stop_evt.set()
        if self._worker_thread is not None:
            self._worker_thread.join(timeout=2.0)
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

    def _enqueue_command(self, cmd: str, *args: Any) -> None:
        self._command_queue.put((cmd, args))

    def _worker_execute_command(self, cmd: str, args: tuple[Any, ...]) -> None:
        try:
            if cmd == 'execute_arm_action':
                msg = args[0]
                result: BackendResult = self._backend.execute_arm_action(msg)
                if result.ok:
                    self._event_queue.put(('status', f'execute mode={msg.control_mode}: {result.message}'))
                else:
                    self._event_queue.put(('error', f'execute mode={msg.control_mode} failed: {result.message}'))
                return

            if cmd == 'stop':
                immediate = bool(args[0])
                result = self._backend.stop(immediate)
                if result.ok:
                    self._event_queue.put(('status', f'stop_arm: {result.message}'))
                else:
                    self._event_queue.put(('error', f'stop_arm failed: {result.message}'))
                return

            if cmd == 'home':
                wait = bool(args[0])
                timeout_sec = float(args[1])
                result = self._backend.home(wait, timeout_sec)
                if result.ok:
                    self._event_queue.put(('status', f'home_arm: {result.message}'))
                else:
                    self._event_queue.put(('error', f'home_arm failed: {result.message}'))
                return

            if cmd == 'reset_fault':
                hard_reset = bool(args[0])
                timeout_sec = float(args[1])
                result = self._backend.reset_fault(hard_reset, timeout_sec)
                if result.ok:
                    self._event_queue.put(('status', f'reset_arm_fault: {result.message}'))
                else:
                    self._event_queue.put(('error', f'reset_arm_fault failed: {result.message}'))
                return

            self._event_queue.put(('error', f'unknown worker command: {cmd}'))
        except Exception as e:
            self._event_queue.put(('error', f'{cmd} exception: {e!s}'))

    def _worker_poll_state(self, now_sec: float) -> None:
        if now_sec < self._state_next_retry_time_sec:
            return

        try:
            if not self._state_publish_enabled:
                conn = self._backend.connect()
                if not conn.ok:
                    raise RuntimeError(conn.message)
                self._state_publish_enabled = True
                self._event_queue.put(('status', conn.message))

            state = self._backend.read_state()
            with self._state_lock:
                self._latest_state = state
            self._state_retry_delay_sec = max(0.1, float(self._state_retry_initial_sec))
            self._state_next_retry_time_sec = 0.0
        except Exception as e:
            self._state_publish_enabled = False
            self._state_next_retry_time_sec = now_sec + self._state_retry_delay_sec
            self._event_queue.put(
                ('error', f'state poll failed: {e!s}; retry in {self._state_retry_delay_sec:.2f}s')
            )
            self._state_retry_delay_sec = min(
                max(self._state_retry_delay_sec * 2.0, 0.1),
                max(float(self._state_retry_max_sec), 0.1),
            )

    def _worker_loop(self) -> None:
        next_state_poll_sec = time.monotonic()
        while not self._worker_stop_evt.is_set():
            now_sec = time.monotonic()
            if self._motion_in_progress:
                try:
                    self._motion_in_progress = bool(self._backend.motion_in_progress())
                except Exception:
                    self._motion_in_progress = False
                self._worker_stop_evt.wait(0.01)
            else:
                wait_sec = max(0.0, next_state_poll_sec - now_sec)
                try:
                    cmd, args = self._command_queue.get(timeout=wait_sec)
                    self._worker_execute_command(cmd, args)
                    try:
                        self._motion_in_progress = bool(self._backend.motion_in_progress())
                    except Exception:
                        self._motion_in_progress = False
                except queue.Empty:
                    pass

            now_sec = time.monotonic()
            if now_sec >= next_state_poll_sec:
                self._worker_poll_state(now_sec)
                next_state_poll_sec = now_sec + self._state_poll_period_sec

    def _on_arm_action(self, msg: ArmAction) -> None:
        if msg.reference_frame == '':
            msg.reference_frame = self._default_reference_frame

        self._last_command_time = self.get_clock().now()
        self._last_timeout_warn_time = None
        self._last_mode = str(msg.control_mode)

        if self._mock_mode:
            result = self._backend.execute_arm_action(msg)
            self._last_result = result.message
            if result.ok:
                self._publish_status(f'execute mode={msg.control_mode}: {result.message}')
            else:
                self._publish_error(f'execute mode={msg.control_mode} failed: {result.message}')
            return

        self._enqueue_command('execute_arm_action', msg)
        self._publish_status(f'motion queued (mode={msg.control_mode})')

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

        self._enqueue_command('stop', bool(request.immediate))
        self._publish_status('stop_arm queued')
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

        self._enqueue_command('home', bool(request.wait), float(request.timeout_sec))
        self._publish_status('home_arm queued')
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

        self._enqueue_command('reset_fault', bool(request.hard_reset), float(request.timeout_sec))
        self._publish_status('reset_arm_fault queued')
        response.success = True
        response.message = 'reset_arm_fault submitted to worker'
        return response

    def _on_status_timer(self) -> None:
        self._drain_worker_events()

        now = self.get_clock().now()
        age_sec = (now - self._last_command_time).nanoseconds / 1e9
        if _finite(age_sec) and age_sec > self._command_timeout_sec:
            should_warn = True
            if self._last_timeout_warn_time is not None:
                since_last_warn = (now - self._last_timeout_warn_time).nanoseconds / 1e9
                if _finite(since_last_warn) and since_last_warn < max(
                    0.1, self._command_timeout_warn_interval_sec
                ):
                    should_warn = False
            if should_warn:
                self.get_logger().warn(
                    f'command timeout: no ArmAction received for {age_sec:.2f}s '
                    f'(limit={self._command_timeout_sec:.2f}s)'
                )
                self._last_timeout_warn_time = now
            return
        self._publish_status(
            f'heartbeat mode={self._last_mode} last_result="{self._last_result}" age={age_sec:.2f}s'
        )

    def _publish_ros_state(self, state: Any) -> None:
        now = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = now
        js.header.frame_id = self._default_reference_frame
        js.position = list(state.joint_position)
        js.velocity = list(state.joint_velocity)
        js.name = self._joint_names if len(self._joint_names) == len(js.position) else []
        self._joint_pub.publish(js)

        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = self._default_reference_frame
        if len(state.ee_translation) >= 3:
            ps.pose.position.x = float(state.ee_translation[0])
            ps.pose.position.y = float(state.ee_translation[1])
            ps.pose.position.z = float(state.ee_translation[2])
        if len(state.ee_quaternion) >= 4:
            ps.pose.orientation.x = float(state.ee_quaternion[0])
            ps.pose.orientation.y = float(state.ee_quaternion[1])
            ps.pose.orientation.z = float(state.ee_quaternion[2])
            ps.pose.orientation.w = float(state.ee_quaternion[3])
        self._pose_pub.publish(ps)

        ts = TwistStamped()
        ts.header.stamp = now
        ts.header.frame_id = self._default_reference_frame
        if len(state.ee_linear_velocity) >= 3:
            ts.twist.linear.x = float(state.ee_linear_velocity[0])
            ts.twist.linear.y = float(state.ee_linear_velocity[1])
            ts.twist.linear.z = float(state.ee_linear_velocity[2])
        if len(state.ee_angular_velocity) >= 3:
            ts.twist.angular.x = float(state.ee_angular_velocity[0])
            ts.twist.angular.y = float(state.ee_angular_velocity[1])
            ts.twist.angular.z = float(state.ee_angular_velocity[2])
        self._twist_pub.publish(ts)

    def _on_state_timer(self) -> None:
        if self._mock_mode:
            return
        with self._state_lock:
            state = self._latest_state
        if state is not None:
            self._publish_ros_state(state)


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
