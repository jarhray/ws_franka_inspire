"""ACT exp3_5 adapter: joint-position control + RT-DETRv4 (same detection as act_exp3_0_det)."""

from __future__ import annotations

from typing import Optional

import numpy as np
from rclpy.node import Node

from robot_interfaces.msg import RobotObservation, WholeBodyAction

from policy_manager.act_exp3_0_det_policy_adapter import ACTExp30DetPolicyAdapter


class ACTExp35DetPolicyAdapter(ACTExp30DetPolicyAdapter):
    """exp3_5 layout: 7 arm joints + hand_grasp_binary; detection/env_state same as act_exp3_0_det."""

    @classmethod
    def from_node(cls, node: Node) -> "ACTExp35DetPolicyAdapter":
        p = "act_exp3_5_det"
        config, det = ACTExp30DetPolicyAdapter._load_configs_from_node(node, p)
        adapter = cls(node=node, config=config, det_config=det)
        node.get_logger().info(
            f"ACT exp3_5 detector policy loaded (joint control): checkpoint={config.checkpoint_dir} "
            f"act_device={adapter._torch_device} detector_enabled={det.enabled} "
            f"detector_device={adapter._detector_device}"
        )
        return adapter

    def _build_state(self, obs: RobotObservation) -> np.ndarray:
        if len(obs.arm_joint_position) != 7:
            raise ValueError(
                f"arm_joint_position size mismatch: got {len(obs.arm_joint_position)}, expected 7"
            )
        state = np.zeros((8,), dtype=np.float32)
        state[0:7] = np.asarray(obs.arm_joint_position, dtype=np.float32)
        state[7] = self._build_obs_hand_binary(obs)
        if self._config.state_dim != 8:
            raise ValueError(f"act_exp3_5_det_state_dim must be 8 for this layout, got {self._config.state_dim}")

        if not self._det_config.enabled or not self._det_config.state_use_detected_xy:
            return state
        try:
            info = self._infer_target_pose_and_scale(obs)
            self._maybe_latch_scale_target(info)
        except Exception as exc:
            if self._det_config.strict_on_failure:
                raise
            if not self._warned_detector_infer:
                self._node.get_logger().warn(
                    f"RT-DETR pose estimation failed, keep joint state unchanged: {exc}"
                )
                self._warned_detector_infer = True
            return state
        self._warned_detector_infer = False
        state[0] = float(info.x)
        state[1] = float(info.y)
        return state

    def _to_whole_body_action(self, action: np.ndarray) -> WholeBodyAction:
        if action.shape != (8,):
            raise ValueError(f"Expected action shape (8,), got {action.shape}")
        msg = WholeBodyAction()
        msg.header.stamp = self._node.get_clock().now().to_msg()

        msg.arm.control_mode = 0
        msg.arm.is_relative = False
        msg.arm.joint_position = [float(x) for x in action[0:7]]
        msg.arm.joint_velocity = []
        msg.arm.duration_sec = float(self._config.arm_duration_sec)

        b = float(action[7])
        use_grasp = b >= float(self._config.hand_binary_threshold)
        hardware = (
            self._config.hand_binary_grasp_hardware
            if use_grasp
            else self._config.hand_binary_reset_hardware
        )
        hand = np.asarray(
            [self._normalized_from_hardware(float(k)) for k in hardware],
            dtype=np.float32,
        )
        msg.hand.control_mode = 0
        msg.hand.is_relative = False
        msg.hand.joint_position = [float(x) for x in hand]
        msg.hand.joint_velocity = []
        msg.hand.duration_sec = float(self._config.hand_duration_sec)

        if not self._det_config.scale_closure_enabled:
            return msg
        cache = self._scale_target_cached
        if cache is None:
            return msg
        grasp_on = float(action[7]) >= float(self._config.hand_binary_threshold)
        if not grasp_on:
            return msg
        if self._det_config.scale_test_log:
            tier = self._scale_tier_label_for_score(cache.size_score)
            self._emit_scale_test_log_throttled(
                "grasp",
                f"[act_exp3_5_det grasp] 档位={tier} | size_score={cache.size_score:.4f} | "
                f"base_xyz_m=({cache.x:.4f}, {cache.y:.4f}, {cache.z:.4f}) | action[7]={float(action[7]):.4f}",
            )
        tier_hw = self._scale_hardware_for_size_score(cache.size_score)
        hand_scaled = np.asarray(
            [self._normalized_from_hardware(float(k)) for k in tier_hw],
            dtype=np.float32,
        )
        msg.hand.joint_position = [float(x) for x in hand_scaled]
        return msg

    def _observation_ready(self, obs: RobotObservation) -> tuple[bool, str]:
        if len(obs.arm_joint_position) != 7:
            return False, "arm_joint_position is not ready (expected 7)."
        return super()._observation_ready(obs)
