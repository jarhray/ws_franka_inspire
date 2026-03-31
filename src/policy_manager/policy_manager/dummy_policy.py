from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from rclpy.node import Node
from robot_interfaces.msg import RobotObservation, WholeBodyAction

from policy_manager.base_policy import BasePolicy


@dataclass(frozen=True)
class DummyPolicyConfig:
    control_hz: float
    arm_control_mode: int
    hand_control_mode: int
    is_relative: bool
    arm_reference_frame: str
    arm_joint_position: list[float]
    arm_joint_velocity: list[float]
    hand_joint_position: list[float]
    hand_joint_velocity: list[float]
    arm_duration_sec: float
    hand_duration_sec: float


class DummyPolicy(BasePolicy):
    def __init__(self, node: Node, config: DummyPolicyConfig) -> None:
        self._node = node
        self._config = config

    @classmethod
    def from_node(cls, node: Node) -> BasePolicy:
        config = DummyPolicyConfig(
            control_hz=float(
                node.declare_parameter("publish_rate_hz", 15.0)
                .get_parameter_value()
                .double_value
            ),
            arm_control_mode=int(
                node.declare_parameter("arm_control_mode", 0)
                .get_parameter_value()
                .integer_value
            ),
            hand_control_mode=int(
                node.declare_parameter("hand_control_mode", 0)
                .get_parameter_value()
                .integer_value
            ),
            is_relative=bool(
                node.declare_parameter("is_relative", False)
                .get_parameter_value()
                .bool_value
            ),
            arm_reference_frame=(
                node.declare_parameter("arm_reference_frame", "fr3_link0")
                .get_parameter_value()
                .string_value
            ),
            arm_joint_position=list(
                node.declare_parameter("arm_joint_position", [0.0] * 7)
                .get_parameter_value()
                .double_array_value
            ),
            arm_joint_velocity=list(
                node.declare_parameter("arm_joint_velocity", [0.0] * 7)
                .get_parameter_value()
                .double_array_value
            ),
            hand_joint_position=list(
                node.declare_parameter("hand_joint_position", [0.0] * 6)
                .get_parameter_value()
                .double_array_value
            ),
            hand_joint_velocity=list(
                node.declare_parameter("hand_joint_velocity", [0.0] * 6)
                .get_parameter_value()
                .double_array_value
            ),
            arm_duration_sec=float(
                node.declare_parameter("arm_duration_sec", 0.1)
                .get_parameter_value()
                .double_value
            ),
            hand_duration_sec=float(
                node.declare_parameter("hand_duration_sec", 0.1)
                .get_parameter_value()
                .double_value
            ),
        )
        return cls(node=node, config=config)

    @property
    def control_hz(self) -> float:
        return float(self._config.control_hz)

    def infer(self, obs: RobotObservation) -> Optional[WholeBodyAction]:
        _ = obs
        msg = WholeBodyAction()
        msg.header.stamp = self._node.get_clock().now().to_msg()

        msg.arm.control_mode = int(self._config.arm_control_mode)
        msg.arm.is_relative = bool(self._config.is_relative)
        msg.arm.reference_frame = self._config.arm_reference_frame
        msg.arm.joint_position = [float(x) for x in self._config.arm_joint_position]
        msg.arm.joint_velocity = [float(x) for x in self._config.arm_joint_velocity]
        msg.arm.duration_sec = float(self._config.arm_duration_sec)

        msg.hand.control_mode = int(self._config.hand_control_mode)
        msg.hand.is_relative = bool(self._config.is_relative)
        msg.hand.joint_position = [float(x) for x in self._config.hand_joint_position]
        msg.hand.joint_velocity = [float(x) for x in self._config.hand_joint_velocity]
        msg.hand.duration_sec = float(self._config.hand_duration_sec)
        return msg
