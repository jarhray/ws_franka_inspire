#!/usr/bin/env python3
"""HandAction.joint_position（[-1,1]）→ SetAngle1（硬件整数 k∈[0,1000]）。"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from robot_interfaces.msg import HandAction
from service_interfaces.msg import SetAngle1

HAND_JOINT_POSITION = 0

def hardware_k_from_normalized_position(x: float) -> int:
    """将模型输出 x∈[-1,1] 线性映射到硬件整数 k∈[0,1000]。"""
    if not math.isfinite(x):
        return 0
    x = max(-1.0, min(1.0, x))
    k = (x + 1.0) * 500.0
    return int(round(k))


class InspireExecutor(Node):
    def __init__(self) -> None:
        super().__init__('inspire_executor')

        self._hand_topic = self.declare_parameter(
            'hand_action_topic', '/robot/hand_action'
        ).get_parameter_value().string_value
        self._set_angle_topic = self.declare_parameter(
            'set_angle_topic', 'set_angle_data'
        ).get_parameter_value().string_value

        self._rate_hz = self.declare_parameter('command_rate_hz', 30.0).get_parameter_value().double_value
        self._hold_last_command = self.declare_parameter('hold_last_command', True).get_parameter_value().bool_value

        self._finger_order: List[int] = list(
            self.declare_parameter(
                'hand_finger_id_order', [0, 1, 2, 3, 4, 5]
            ).get_parameter_value().integer_array_value
        )
        if not self._finger_order:
            self._finger_order = [0, 1, 2, 3, 4, 5]
        self._n_joints = len(self._finger_order)

        self._pub = self.create_publisher(SetAngle1, self._set_angle_topic, 10)
        self.create_subscription(HandAction, self._hand_topic, self._on_hand, 10)

        self._last_cmd: Optional[HandAction] = None
        self._has_cmd = False

        period = 1.0 / max(0.1, self._rate_hz)
        self._timer = None
        if self._hold_last_command:
            self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'inspire_executor: sub={self._hand_topic} pub={self._set_angle_topic} '
            f'rate={self._rate_hz} Hz fingers={self._finger_order}'
        )

    def _on_hand(self, msg: HandAction) -> None:
        self._last_cmd = msg
        self._has_cmd = True
        if not self._hold_last_command:
            self._publish_set_angle(msg)

    def _on_timer(self) -> None:
        if not self._has_cmd or self._last_cmd is None:
            return
        if self._hold_last_command:
            self._publish_set_angle(self._last_cmd)

    def _publish_set_angle(self, msg: HandAction) -> None:
        if msg.control_mode != HAND_JOINT_POSITION:
            return

        pos = list(msg.joint_position)
        if len(pos) < self._n_joints:
            pos = pos + [0.0] * (self._n_joints - len(pos))
        elif len(pos) > self._n_joints:
            pos = pos[: self._n_joints]

        angles: List[int] = [hardware_k_from_normalized_position(x) for x in pos]

        out = SetAngle1()
        out.finger_ids = [int(x) for x in self._finger_order]
        out.angles = angles

        self._pub.publish(out)


def main(args: Optional[Tuple[str, ...]] = None) -> None:
    rclpy.init(args=args)
    node = InspireExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
