#!/usr/bin/env python3
"""Policy manager with policy_type switch and ACT integration."""

from __future__ import annotations

import sys
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from robot_interfaces.msg import RobotObservation, WholeBodyAction
from policy_manager.base_policy import BasePolicy
from policy_manager.act_policy_adapter import ACTPolicyAdapter
from policy_manager.act_exp1_policy_adapter import ACTExp1PolicyAdapter
from policy_manager.act_exp1_1_policy_adapter import ACTExp11PolicyAdapter
from policy_manager.act_exp1_2_policy_adapter import ACTExp12PolicyAdapter
from policy_manager.act_exp1_3_policy_adapter import ACTExp13PolicyAdapter
from policy_manager.act_exp3_0_policy_adapter import ACTExp30PolicyAdapter
from policy_manager.act_exp3_0_det_policy_adapter import ACTExp30DetPolicyAdapter
from policy_manager.act_exp3_5_det_policy_adapter import ACTExp35DetPolicyAdapter
from policy_manager.act_exp3_1_policy_adapter import ACTExp31PolicyAdapter
from policy_manager.act_exp3_2_policy_adapter import ACTExp32PolicyAdapter
from policy_manager.pi05_exp1_3_lora_policy_adapter import PI05Exp13LoraPolicyAdapter
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
        self._act_strict_on_error = (
            self.declare_parameter('act_strict_on_error', True)
            .get_parameter_value()
            .bool_value
        )

        self.create_subscription(RobotObservation, self._observation_topic, self._on_observation, 10)
        self._pub = self.create_publisher(WholeBodyAction, self._whole_body_action_topic, 10)
        if self._policy_type not in (
            'dummy',
            'bc',
            'vla',
            'act',
            'act_exp1',
            'act_exp1_1',
            'act_exp1_2',
            'act_exp1_3',
            'act_exp3_0',
            'act_exp3_0_det',
            'act_exp3_5_det',
            'act_exp3_1',
            'act_exp3_2',
            'pi05_exp1_3_lora',
        ):
            raise RuntimeError(f'Unknown policy_type={self._policy_type}.')

        # 根据 policy_type 选择具体模型，并让模型自己声明/读取所需参数。
        if self._policy_type == 'act':
            self._policy = ACTPolicyAdapter.from_node(self)
        elif self._policy_type == 'act_exp1':
            self._policy = ACTExp1PolicyAdapter.from_node(self)
        elif self._policy_type == 'act_exp1_1':
            self._policy = ACTExp11PolicyAdapter.from_node(self)
        elif self._policy_type == 'act_exp1_2':
            self._policy = ACTExp12PolicyAdapter.from_node(self)
        elif self._policy_type == 'act_exp1_3':
            self._policy = ACTExp13PolicyAdapter.from_node(self)
        elif self._policy_type == 'act_exp3_0':
            self._policy = ACTExp30PolicyAdapter.from_node(self)
        elif self._policy_type == 'act_exp3_0_det':
            self._policy = ACTExp30DetPolicyAdapter.from_node(self)
        elif self._policy_type == 'act_exp3_5_det':
            self._policy = ACTExp35DetPolicyAdapter.from_node(self)
        elif self._policy_type == 'act_exp3_1':
            self._policy = ACTExp31PolicyAdapter.from_node(self)
        elif self._policy_type == 'act_exp3_2':
            self._policy = ACTExp32PolicyAdapter.from_node(self)
        elif self._policy_type == 'pi05_exp1_3_lora':
            self._policy = PI05Exp13LoraPolicyAdapter.from_node(self)
        elif self._policy_type == 'dummy':
            self._policy = DummyPolicy.from_node(self)
        elif self._policy_type in ('bc', 'vla'):
            self.get_logger().warn(
                f'policy_type={self._policy_type} not integrated; using dummy policy parameters.'
            )
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
        except ValueError as exc:
            # 观测维度/图像等与模型不一致：跳过本周期，不退出节点。
            if not self._warned_bad_observation:
                self.get_logger().warn(f'Skip policy inference (bad observation): {exc}')
                self._warned_bad_observation = True
            return None
        except Exception as exc:
            # ACT 严格模式：加载已成功后，除 ValueError 外的推理错误视为致命（GPU/模型等）。
            if self._policy_type == 'act' and self._act_strict_on_error:
                self.get_logger().fatal(f'ACT inference failed (strict exit): {exc}')
                try:
                    self.destroy_node()
                finally:
                    rclpy.shutdown()
                sys.exit(1)
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
