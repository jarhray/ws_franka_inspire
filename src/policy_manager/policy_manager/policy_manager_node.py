#!/usr/bin/env python3
"""Policy manager with policy_type switch and dummy fallback output."""

from __future__ import annotations

from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from robot_interfaces.msg import RobotObservation, WholeBodyAction


class PolicyManager(Node):
    def __init__(self) -> None:
        super().__init__('policy_manager')

        self._policy_type = self.declare_parameter('policy_type', 'dummy').get_parameter_value().string_value
        self._publish_rate_hz = self.declare_parameter(
            'publish_rate_hz', 15.0
        ).get_parameter_value().double_value
        self._observation_topic = self.declare_parameter(
            'observation_topic', '/robot/observation'
        ).get_parameter_value().string_value
        self._whole_body_action_topic = self.declare_parameter(
            'whole_body_action_topic', '/robot/whole_body_action'
        ).get_parameter_value().string_value
        self._require_observation = self.declare_parameter(
            'require_observation_before_publish', True
        ).get_parameter_value().bool_value

        self._arm_control_mode = self.declare_parameter('arm_control_mode', 0).get_parameter_value().integer_value
        self._hand_control_mode = self.declare_parameter(
            'hand_control_mode', 0
        ).get_parameter_value().integer_value
        self._is_relative = self.declare_parameter('is_relative', False).get_parameter_value().bool_value
        self._arm_reference_frame = self.declare_parameter(
            'arm_reference_frame', 'fr3_link0'
        ).get_parameter_value().string_value

        self._arm_joint_position = list(
            self.declare_parameter('arm_joint_position', [0.0] * 7).get_parameter_value().double_array_value
        )
        self._arm_joint_velocity = list(
            self.declare_parameter('arm_joint_velocity', [0.0] * 7).get_parameter_value().double_array_value
        )
        self._hand_joint_position = list(
            self.declare_parameter('hand_joint_position', [0.0] * 6).get_parameter_value().double_array_value
        )
        self._hand_joint_velocity = list(
            self.declare_parameter('hand_joint_velocity', [0.0] * 6).get_parameter_value().double_array_value
        )
        self._arm_duration_sec = self.declare_parameter(
            'arm_duration_sec', 0.1
        ).get_parameter_value().double_value
        self._hand_duration_sec = self.declare_parameter(
            'hand_duration_sec', 0.1
        ).get_parameter_value().double_value

        self._last_observation: Optional[RobotObservation] = None
        self._warned_no_observation = False

        self.create_subscription(RobotObservation, self._observation_topic, self._on_observation, 10)
        self._pub = self.create_publisher(WholeBodyAction, self._whole_body_action_topic, 10)
        self.create_timer(1.0 / max(self._publish_rate_hz, 0.1), self._on_timer)

        if self._policy_type not in ('dummy', 'bc', 'vla'):
            self.get_logger().warn(
                f'Unknown policy_type={self._policy_type}, fallback to dummy.'
            )
            self._policy_type = 'dummy'

        self.get_logger().info(
            f'policy_manager: type={self._policy_type} obs={self._observation_topic} '
            f'out={self._whole_body_action_topic} rate={self._publish_rate_hz}Hz'
        )
        if self._policy_type in ('bc', 'vla'):
            self.get_logger().warn(
                f'policy_type={self._policy_type} not integrated yet; using dummy outputs.'
            )

    def _on_observation(self, msg: RobotObservation) -> None:
        self._last_observation = msg
        self._warned_no_observation = False

    def _build_dummy_action(self) -> WholeBodyAction:
        msg = WholeBodyAction()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.arm.control_mode = int(self._arm_control_mode)
        msg.arm.is_relative = bool(self._is_relative)
        msg.arm.reference_frame = self._arm_reference_frame
        msg.arm.joint_position = [float(x) for x in self._arm_joint_position]
        msg.arm.joint_velocity = [float(x) for x in self._arm_joint_velocity]
        msg.arm.duration_sec = float(self._arm_duration_sec)

        msg.hand.control_mode = int(self._hand_control_mode)
        msg.hand.is_relative = bool(self._is_relative)
        msg.hand.joint_position = [float(x) for x in self._hand_joint_position]
        msg.hand.joint_velocity = [float(x) for x in self._hand_joint_velocity]
        msg.hand.duration_sec = float(self._hand_duration_sec)
        return msg

    def _on_timer(self) -> None:
        if self._require_observation and self._last_observation is None:
            if not self._warned_no_observation:
                self.get_logger().warn(
                    'No RobotObservation received yet; policy output is paused.'
                )
                self._warned_no_observation = True
            return

        if self._policy_type == 'dummy':
            out = self._build_dummy_action()
        elif self._policy_type in ('bc', 'vla'):
            # Placeholder: keep interface stable while model integration is pending.
            out = self._build_dummy_action()
        else:
            out = self._build_dummy_action()

        self._pub.publish(out)


def main(args: Optional[Tuple[str, ...]] = None) -> None:
    rclpy.init(args=args)
    node = PolicyManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
