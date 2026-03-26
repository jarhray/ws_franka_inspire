#!/usr/bin/env python3
"""Maps robot_interfaces/HandAction to service_interfaces/SetAngle1 for inspire_hand_modbus_ros2."""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from robot_interfaces.msg import HandAction
from service_interfaces.msg import SetAngle1

HAND_JOINT_POSITION = 0


def _finite(x: float) -> bool:
    return math.isfinite(x)


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


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

        default_min = [0.0] * self._n_joints
        default_max = [1000.0] * self._n_joints
        self._joint_min = list(
            self.declare_parameter('joint_position_min', default_min).get_parameter_value().double_array_value
        )
        self._joint_max = list(
            self.declare_parameter('joint_position_max', default_max).get_parameter_value().double_array_value
        )

        self._input_normalized_0_1 = self.declare_parameter(
            'input_normalized_0_1', False
        ).get_parameter_value().bool_value

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

    def _scale_positions(self, positions: List[float]) -> List[float]:
        out: List[float] = []
        for p in positions:
            if not _finite(p):
                out.append(0.0)
                continue
            v = p * 1000.0 if self._input_normalized_0_1 else p
            out.append(v)
        return out

    def _clamp_row(self, positions: List[float]) -> List[int]:
        if len(self._joint_min) != self._n_joints or len(self._joint_max) != self._n_joints:
            self.get_logger().warn('joint_position_min/max length mismatch; skipping clamp.')
            return [int(round(p)) for p in positions]

        ints: List[int] = []
        for i, p in enumerate(positions):
            lo = self._joint_min[i]
            hi = self._joint_max[i]
            if _finite(p) and _finite(lo) and _finite(hi):
                c = _clamp(p, lo, hi)
            else:
                c = 0.0
            ints.append(int(round(c)))
        return ints

    def _publish_set_angle(self, msg: HandAction) -> None:
        if msg.control_mode != HAND_JOINT_POSITION:
            return

        pos = list(msg.joint_position)
        if len(pos) < self._n_joints:
            pos = pos + [0.0] * (self._n_joints - len(pos))
        elif len(pos) > self._n_joints:
            pos = pos[: self._n_joints]

        scaled = self._scale_positions(pos)
        clamped = self._clamp_row(scaled)

        out = SetAngle1()
        out.finger_ids = [int(x) for x in self._finger_order]
        out.angles = clamped

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
