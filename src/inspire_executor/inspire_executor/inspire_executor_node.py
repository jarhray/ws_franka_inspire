#!/usr/bin/env python3
"""HandAction.joint_position（弧度 r）→ SetAngle1（硬件整数 k∈[0,1000]）。"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node

from robot_interfaces.msg import HandAction
from service_interfaces.msg import SetAngle1

HAND_JOINT_POSITION = 0

# ---------------------------------------------------------------------------
# 标定：r = f(k)，k 为硬件 0–1000。与 observation_aggregator 侧「k→r」一致。
# 逆映射：预计算 f(0)..f(1000)，对 r 在表上做二分（每关节 O(log N)，N=1001，无热路径三次幂）。
# finger_id：0–3 四指弯曲，4 拇指弯曲，5 拇指侧摆（闭式逆，O(1)）。
# ---------------------------------------------------------------------------


def radians_from_hardware_four_fingers(k: float) -> float:
    return -5e-10 * k**3 + 9e-7 * k**2 - 0.0018 * k + 1.4191


def radians_from_hardware_thumb_flexion(k: float) -> float:
    return 8e-11 * k**3 - 4e-8 * k**2 - 0.0006 * k + 0.5869


def radians_from_hardware_thumb_abduction(k: float) -> float:
    return -0.0012 * k + 1.1641


def _build_table(fn, n: int = 1001) -> List[float]:
    return [fn(float(k)) for k in range(n)]


_R_TABLE_FOUR = _build_table(radians_from_hardware_four_fingers)
_R_TABLE_THUMB_FLEX = _build_table(radians_from_hardware_thumb_flexion)


def _hardware_k_thumb_abduction_from_radians(r: float) -> int:
    """r = -0.0012*k + 1.1641 的闭式逆。"""
    k = (1.1641 - r) / 0.0012
    return int(max(0, min(1000, round(k))))


def _hardware_k_from_radians_and_table(r: float, table: Sequence[float]) -> int:
    """
    table[k]=f(k) 在 [0,1000] 上单调递减时，求最接近目标的整数 k。
    复杂度：O(log N) 比较，N=1001。
    """
    if not math.isfinite(r):
        return 0
    if r >= table[0]:
        return 0
    if r <= table[1000]:
        return 1000
    lo, hi = 0, 1000
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if table[mid] > r:
            lo = mid
        else:
            hi = mid
    if abs(table[lo] - r) <= abs(table[hi] - r):
        return int(lo)
    return int(hi)


def hardware_k_from_radians(r: float, finger_id: int) -> int:
    if finger_id in (0, 1, 2, 3):
        return _hardware_k_from_radians_and_table(r, _R_TABLE_FOUR)
    if finger_id == 4:
        return _hardware_k_from_radians_and_table(r, _R_TABLE_THUMB_FLEX)
    if finger_id == 5:
        return _hardware_k_thumb_abduction_from_radians(r)
    return _hardware_k_from_radians_and_table(r, _R_TABLE_FOUR)


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

        angles: List[int] = []
        for i, r in enumerate(pos):
            fid = int(self._finger_order[i])
            angles.append(hardware_k_from_radians(r, fid))

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
