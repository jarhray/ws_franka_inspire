#!/usr/bin/env python3
"""Policy manager with policy_type switch and ACT integration."""

from __future__ import annotations

from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from robot_interfaces.msg import RobotObservation, WholeBodyAction
from policy_manager.base_policy import BasePolicy
from policy_manager.act_policy_adapter import ACTPolicyAdapter
from policy_manager.dummy_policy import DummyPolicy


class PolicyManager(Node):
    def __init__(self) -> None:
        super().__init__('policy_manager')

        self._policy_type = self.declare_parameter('policy_type', 'dummy').get_parameter_value().string_value
        self._observation_topic = self.declare_parameter(
            'observation_topic', '/robot/observation'
        ).get_parameter_value().string_value
        self._whole_body_action_topic = self.declare_parameter(
            'whole_body_action_topic', '/robot/whole_body_action'
        ).get_parameter_value().string_value
        self._require_observation = self.declare_parameter(
            'require_observation_before_publish', True
        ).get_parameter_value().bool_value

        self._last_observation: Optional[RobotObservation] = None
        self._warned_no_observation = False
        self._policy: Optional[BasePolicy] = None
        self._warned_bad_observation = False

        self.create_subscription(RobotObservation, self._observation_topic, self._on_observation, 10)
        self._pub = self.create_publisher(WholeBodyAction, self._whole_body_action_topic, 10)
        if self._policy_type not in ('dummy', 'bc', 'vla', 'act'):
            raise RuntimeError(f'Unknown policy_type={self._policy_type}.')

        # 根据 policy_type 选择具体模型，并让模型自己声明/读取所需参数。
        if self._policy_type == 'act':
            self._policy = ACTPolicyAdapter.from_node(self)
        elif self._policy_type == 'dummy':
            self._policy = DummyPolicy.from_node(self)
        else:
            self.get_logger().warn(
                f'policy_type={self._policy_type} not integrated yet; using no-op policy.'
            )
            self._policy = None

        timer_hz = self._policy.control_hz if self._policy is not None else 1.0
        self.create_timer(1.0 / max(timer_hz, 0.1), self._on_timer)

        self.get_logger().info(
            f'policy_manager: type={self._policy_type} obs={self._observation_topic} '
            f'out={self._whole_body_action_topic} rate={timer_hz}Hz'
        )
        if self._policy is None:
            self.get_logger().warn('No concrete policy initialized; node will not publish actions.')

    def _on_observation(self, msg: RobotObservation) -> None:
        self._last_observation = msg
        self._warned_no_observation = False

    def _infer_policy_action(self, obs: RobotObservation) -> Optional[WholeBodyAction]:
        try:
            if self._policy is None:
                return None
            msg = self._policy.infer(obs)
        except Exception as exc:
            if not self._warned_bad_observation:
                self.get_logger().warn(f'Skip policy inference due to error: {exc}')
                self._warned_bad_observation = True
            return None
        self._warned_bad_observation = False
        return msg

    def _on_timer(self) -> None:
        if self._require_observation and self._last_observation is None:
            if not self._warned_no_observation:
                self.get_logger().warn(
                    'No RobotObservation received yet; policy output is paused.'
                )
                self._warned_no_observation = True
            return

        out = self._infer_policy_action(self._last_observation)
        if out is None:
            return

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
