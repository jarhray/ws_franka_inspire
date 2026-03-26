#!/usr/bin/env python3
"""Routes WholeBodyAction to standardized ArmAction / HandAction with safety and relative handling."""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Quaternion, Twist, Vector3
from robot_interfaces.msg import ArmAction, HandAction, RobotObservation, WholeBodyAction


# ArmAction.control_mode
ARM_JOINT_POSITION = 0
ARM_JOINT_VELOCITY = 1
ARM_CARTESIAN_POSE = 2
ARM_CARTESIAN_VELOCITY = 3

# HandAction.control_mode
HAND_JOINT_POSITION = 0


def _finite(x: float) -> bool:
    return math.isfinite(x)


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _quat_norm(q: Quaternion) -> float:
    return math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)


def _quat_normalize(q: Quaternion) -> Quaternion:
    n = _quat_norm(q)
    if n < 1e-9:
        out = Quaternion()
        out.w = 1.0
        return out
    inv = 1.0 / n
    out = Quaternion()
    out.x = q.x * inv
    out.y = q.y * inv
    out.z = q.z * inv
    out.w = q.w * inv
    return out


def _quat_mult(a: Quaternion, b: Quaternion) -> Quaternion:
    """Hamilton product a * b (geometry_msgs x,y,z,w)."""
    ax, ay, az, aw = a.x, a.y, a.z, a.w
    bx, by, bz, bw = b.x, b.y, b.z, b.w
    out = Quaternion()
    out.w = aw * bw - ax * bx - ay * by - az * bz
    out.x = aw * bx + ax * bw + ay * bz - az * by
    out.y = aw * by - ax * bz + ay * bw + az * bx
    out.z = aw * bz + ax * by - ay * bx + az * bw
    return _quat_normalize(out)


def _twist_add(a: Twist, b: Twist) -> Twist:
    out = Twist()
    out.linear.x = a.linear.x + b.linear.x
    out.linear.y = a.linear.y + b.linear.y
    out.linear.z = a.linear.z + b.linear.z
    out.angular.x = a.angular.x + b.angular.x
    out.angular.y = a.angular.y + b.angular.y
    out.angular.z = a.angular.z + b.angular.z
    return out


def _vec3_clamp_mag(v: Vector3, max_mag: float) -> Vector3:
    if max_mag <= 0.0 or not _finite(max_mag):
        return v
    lx, ly, lz = v.x, v.y, v.z
    m = math.sqrt(lx * lx + ly * ly + lz * lz)
    if m <= max_mag or m < 1e-12:
        return v
    s = max_mag / m
    out = Vector3()
    out.x = lx * s
    out.y = ly * s
    out.z = lz * s
    return out


class ActionRouter(Node):
    def __init__(self) -> None:
        super().__init__('action_router')

        self._whole_body_topic = self.declare_parameter(
            'whole_body_action_topic', '/robot/whole_body_action'
        ).get_parameter_value().string_value
        self._obs_topic = self.declare_parameter(
            'observation_topic', '/robot/observation'
        ).get_parameter_value().string_value
        self._arm_out_topic = self.declare_parameter(
            'arm_action_topic', '/robot/arm_action'
        ).get_parameter_value().string_value
        self._hand_out_topic = self.declare_parameter(
            'hand_action_topic', '/robot/hand_action'
        ).get_parameter_value().string_value

        self._arm_joint_count = self.declare_parameter('arm_joint_count', 7).get_parameter_value().integer_value
        self._hand_joint_count = self.declare_parameter('hand_joint_count', 6).get_parameter_value().integer_value

        self._arm_joint_pos_min = list(
            self.declare_parameter('arm_joint_position_min', [-2.9] * 7).get_parameter_value().double_array_value
        )
        self._arm_joint_pos_max = list(
            self.declare_parameter('arm_joint_position_max', [2.9] * 7).get_parameter_value().double_array_value
        )
        self._arm_joint_vel_max = self.declare_parameter(
            'arm_joint_velocity_limit', 2.0
        ).get_parameter_value().double_value

        self._cart_vel_lin_max = self.declare_parameter(
            'cartesian_linear_velocity_max', 1.0
        ).get_parameter_value().double_value
        self._cart_vel_ang_max = self.declare_parameter(
            'cartesian_angular_velocity_max', 1.0
        ).get_parameter_value().double_value

        self._duration_min = self.declare_parameter('duration_sec_min', 0.0).get_parameter_value().double_value
        self._duration_max = self.declare_parameter('duration_sec_max', 60.0).get_parameter_value().double_value

        self._hand_pos_min = list(
            self.declare_parameter('hand_joint_position_min', [0.0] * 6).get_parameter_value().double_array_value
        )
        self._hand_pos_max = list(
            self.declare_parameter('hand_joint_position_max', [1000.0] * 6).get_parameter_value().double_array_value
        )

        self._last_obs: Optional[RobotObservation] = None
        self._warned_no_obs = False

        self._sub_wb = self.create_subscription(
            WholeBodyAction, self._whole_body_topic, self._on_whole_body, 10
        )
        self._sub_obs = self.create_subscription(
            RobotObservation, self._obs_topic, self._on_obs, 10
        )
        self._pub_arm = self.create_publisher(ArmAction, self._arm_out_topic, 10)
        self._pub_hand = self.create_publisher(HandAction, self._hand_out_topic, 10)

        self.get_logger().info(
            f'action_router: in={self._whole_body_topic} obs={self._obs_topic} '
            f'arm_out={self._arm_out_topic} hand_out={self._hand_out_topic}'
        )

    def _on_obs(self, msg: RobotObservation) -> None:
        self._last_obs = msg
        self._warned_no_obs = False

    def _on_whole_body(self, msg: WholeBodyAction) -> None:
        stamp = msg.header.stamp if msg.header.stamp.sec or msg.header.stamp.nanosec else self.get_clock().now().to_msg()

        arm_out, arm_ok = self._process_arm(msg.arm, stamp)
        hand_out, hand_ok = self._process_hand(msg.hand, stamp)

        if arm_ok:
            self._pub_arm.publish(arm_out)
        if hand_ok:
            self._pub_hand.publish(hand_out)

    def _need_obs(self, is_relative: bool) -> bool:
        if not is_relative:
            return True
        if self._last_obs is None:
            if not self._warned_no_obs:
                self.get_logger().warn(
                    'Relative action requires RobotObservation; skipping until observation is received.'
                )
                self._warned_no_obs = True
            return False
        return True

    def _process_arm(self, arm: ArmAction, stamp) -> Tuple[ArmAction, bool]:
        out = ArmAction()
        out.header.stamp = stamp
        out.header.frame_id = arm.header.frame_id
        out.control_mode = arm.control_mode
        out.is_relative = False
        out.reference_frame = arm.reference_frame
        out.duration_sec = self._clamp_duration(arm.duration_sec)

        if not self._need_obs(arm.is_relative):
            return out, False

        obs = self._last_obs

        if arm.control_mode == ARM_JOINT_POSITION:
            jp = list(arm.joint_position)
            if arm.is_relative and obs is not None:
                base = self._resize_vec(list(obs.arm_joint_position), self._arm_joint_count)
                jp = self._resize_vec(jp, self._arm_joint_count)
                jp = [a + b for a, b in zip(jp, base)]
            else:
                jp = self._resize_vec(jp, self._arm_joint_count)
            out.joint_position = self._clamp_arm_joint_positions(jp)

        elif arm.control_mode == ARM_JOINT_VELOCITY:
            jv = list(arm.joint_velocity)
            if arm.is_relative and obs is not None:
                base = self._resize_vec(list(obs.arm_joint_velocity), self._arm_joint_count)
                jv = self._resize_vec(jv, self._arm_joint_count)
                jv = [a + b for a, b in zip(jv, base)]
            else:
                jv = self._resize_vec(jv, self._arm_joint_count)
            out.joint_velocity = self._clamp_arm_joint_velocities(jv)

        elif arm.control_mode == ARM_CARTESIAN_POSE:
            pose = arm.cartesian_pose
            if arm.is_relative and obs is not None:
                pose.position.x = obs.ee_pose.position.x + pose.position.x
                pose.position.y = obs.ee_pose.position.y + pose.position.y
                pose.position.z = obs.ee_pose.position.z + pose.position.z
                q_obs = _quat_normalize(obs.ee_pose.orientation)
                q_delta = _quat_normalize(pose.orientation)
                pose.orientation = _quat_mult(q_obs, q_delta)
            out.cartesian_pose = pose

        elif arm.control_mode == ARM_CARTESIAN_VELOCITY:
            tw = arm.cartesian_velocity
            if arm.is_relative and obs is not None:
                tw = _twist_add(obs.ee_twist, tw)
            tw.linear = _vec3_clamp_mag(tw.linear, self._cart_vel_lin_max)
            tw.angular = _vec3_clamp_mag(tw.angular, self._cart_vel_ang_max)
            out.cartesian_velocity = tw

        else:
            self.get_logger().warn(f'Unknown arm control_mode={arm.control_mode}; passing through fields.')
            out.joint_position = list(arm.joint_position)
            out.joint_velocity = list(arm.joint_velocity)
            out.cartesian_pose = arm.cartesian_pose
            out.cartesian_velocity = arm.cartesian_velocity

        return out, True

    def _process_hand(self, hand: HandAction, stamp) -> Tuple[HandAction, bool]:
        out = HandAction()
        out.header.stamp = stamp
        out.header.frame_id = hand.header.frame_id
        out.control_mode = hand.control_mode
        out.is_relative = False
        out.duration_sec = self._clamp_duration(hand.duration_sec)

        if hand.control_mode != HAND_JOINT_POSITION:
            self.get_logger().warn(
                f'Hand control_mode={hand.control_mode} not supported; only joint_position (0) is implemented.'
            )
            return out, False

        if not self._need_obs(hand.is_relative):
            return out, False

        obs = self._last_obs
        jp = list(hand.joint_position)
        if hand.is_relative and obs is not None:
            base = self._resize_vec(list(obs.hand_joint_position), self._hand_joint_count)
            jp = self._resize_vec(jp, self._hand_joint_count)
            jp = [a + b for a, b in zip(jp, base)]
        else:
            jp = self._resize_vec(jp, self._hand_joint_count)
        out.joint_position = self._clamp_hand_positions(jp)
        out.joint_velocity = []
        return out, True

    def _clamp_duration(self, d: float) -> float:
        if not _finite(d):
            return 0.0
        return _clamp(d, self._duration_min, self._duration_max)

    def _resize_vec(self, v: List[float], n: int) -> List[float]:
        if len(v) >= n:
            return [float(v[i]) for i in range(n)]
        return [float(v[i]) if i < len(v) else 0.0 for i in range(n)]

    def _clamp_arm_joint_positions(self, jp: List[float]) -> List[float]:
        if not self._arm_joint_pos_min or not self._arm_joint_pos_max:
            return jp
        if len(self._arm_joint_pos_min) != len(jp) or len(self._arm_joint_pos_max) != len(jp):
            return jp
        out = []
        for i, x in enumerate(jp):
            lo = self._arm_joint_pos_min[i]
            hi = self._arm_joint_pos_max[i]
            if _finite(x) and _finite(lo) and _finite(hi):
                out.append(_clamp(x, lo, hi))
            else:
                out.append(0.0)
        return out

    def _clamp_arm_joint_velocities(self, jv: List[float]) -> List[float]:
        lim = self._arm_joint_vel_max
        if not _finite(lim) or lim <= 0.0:
            return jv
        out = []
        for x in jv:
            if not _finite(x):
                out.append(0.0)
            else:
                out.append(_clamp(x, -lim, lim))
        return out

    def _clamp_hand_positions(self, jp: List[float]) -> List[float]:
        if not self._hand_pos_min or not self._hand_pos_max:
            return jp
        if len(self._hand_pos_min) != len(jp) or len(self._hand_pos_max) != len(jp):
            return jp
        out = []
        for i, x in enumerate(jp):
            lo = self._hand_pos_min[i]
            hi = self._hand_pos_max[i]
            if _finite(x) and _finite(lo) and _finite(hi):
                out.append(_clamp(x, lo, hi))
            else:
                out.append(_clamp(0.0, lo, hi))
        return out


def main(args: Optional[Tuple[str, ...]] = None) -> None:
    rclpy.init(args=args)
    node = ActionRouter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
