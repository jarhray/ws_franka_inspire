"""ACT policy adapter for exp1_lerobot dataset layout (30-dim state, 13-dim joint action)."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
import torch
import torch.nn.functional as F
from rclpy.node import Node
from sensor_msgs.msg import Image

from robot_interfaces.msg import RobotObservation, WholeBodyAction

from policy_manager.base_policy import BasePolicy
from policy_manager.observation_camera import resolve_observation_images


# ---------------------------------------------------------------------------
# Inspire：硬件 k∈[0,1000] → 弧度 r = f(k)。与 observation_aggregator 中定义保持一致。
# 手部策略输出为弧度时，用 f(0) 与 f(1000) 作为上下界做线性映射到 [-1,1]，供 inspire_executor。
# ---------------------------------------------------------------------------


def _radians_from_hardware_four_fingers(k: float) -> float:
    return -5e-10 * k**3 + 9e-7 * k**2 - 0.0018 * k + 1.4191


def _radians_from_hardware_thumb_flexion(k: float) -> float:
    return 8e-11 * k**3 - 4e-8 * k**2 - 0.0006 * k + 0.5869


def _radians_from_hardware_thumb_abduction(k: float) -> float:
    return -0.0012 * k + 1.1641


def _radians_from_hardware_for_finger_id(k: float, finger_id: int) -> float:
    if finger_id in (0, 1, 2, 3):
        return _radians_from_hardware_four_fingers(k)
    if finger_id == 4:
        return _radians_from_hardware_thumb_flexion(k)
    if finger_id == 5:
        return _radians_from_hardware_thumb_abduction(k)
    return _radians_from_hardware_four_fingers(k)


def _normalized_position_from_radians(r: float, finger_id: int) -> float:
    """弧度 r -> [-1,1]：在 k=0 与 k=1000 对应的弧度区间内做线性映射。"""
    if not np.isfinite(r):
        return 0.0
    r0 = _radians_from_hardware_for_finger_id(0.0, finger_id)
    r1 = _radians_from_hardware_for_finger_id(1000.0, finger_id)
    lo = float(min(r0, r1))
    hi = float(max(r0, r1))
    if hi <= lo:
        return 0.0
    x = 2.0 * (float(r) - lo) / (hi - lo) - 1.0
    return float(max(-1.0, min(1.0, x)))


@dataclass(frozen=True)
class ACTExp1PolicyConfig:
    checkpoint_dir: str
    device: str
    image_key: str
    depth_key: str
    use_depth: bool
    state_key: str
    img_h: int
    img_w: int
    state_dim: int
    action_dim: int
    touch_dim: int
    arm_reference_frame: str
    arm_duration_sec: float
    hand_duration_sec: float
    control_hz: float
    hand_finger_id_order: tuple[int, int, int, int, int, int]
    obs_camera_id: str = ""
    obs_depth_camera_id: str = ""


class ACTExp1PolicyAdapter(BasePolicy):
    """Matches exp1_lerobot/meta/info.json feature layout.

    observation.state (30): 7 arm joints + 7 ee pose + 6 hand angles + touch_forces (10).
    Touch: concatenate ``hand_touch.normal_forces`` and ``hand_touch.tangential_forces``,
    then take the first ``touch_dim`` scalars (default 10) to align with training.

    Images: ``observation.images.rs_color`` (RGB) and ``observation.rs_depth`` (3-channel
    float in [0, 1], built from ``depth_image`` — see ``_depth_to_numpy_hwc3``).

    action (13): 7 arm joint targets + 6 hand angles (弧度) -> WholeBodyAction；
    手部在消息中转为 [-1,1]（与 inspire_executor 对 HandAction 的约定一致）。
    """

    def __init__(self, node: Node, config: ACTExp1PolicyConfig) -> None:
        self._node = node
        self._config = config
        self._torch_device = self._resolve_torch_device(config.device)
        self._policy = self._load_policy(config.checkpoint_dir)
        self._warned_touch_shape = False

    @property
    def torch_device(self) -> str:
        return self._torch_device

    @property
    def control_hz(self) -> float:
        return float(self._config.control_hz)

    @classmethod
    def from_node(cls, node: Node) -> "BasePolicy":
        checkpoint_dir = (
            node.declare_parameter("act_exp1_checkpoint_dir", "")
            .get_parameter_value()
            .string_value
        )
        device = (
            node.declare_parameter("act_exp1_device", "auto")
            .get_parameter_value()
            .string_value
        )
        image_key = (
            node.declare_parameter(
                "act_exp1_image_key", "observation.images.rs_color"
            )
            .get_parameter_value()
            .string_value
        )
        depth_key = (
            node.declare_parameter("act_exp1_depth_key", "observation.rs_depth")
            .get_parameter_value()
            .string_value
        )
        use_depth = (
            node.declare_parameter("act_exp1_use_depth", True)
            .get_parameter_value()
            .bool_value
        )
        state_key = (
            node.declare_parameter("act_exp1_state_key", "observation.state")
            .get_parameter_value()
            .string_value
        )
        img_h = (
            node.declare_parameter("act_exp1_img_h", 480)
            .get_parameter_value()
            .integer_value
        )
        img_w = (
            node.declare_parameter("act_exp1_img_w", 640)
            .get_parameter_value()
            .integer_value
        )
        state_dim = (
            node.declare_parameter("act_exp1_state_dim", 30)
            .get_parameter_value()
            .integer_value
        )
        action_dim = (
            node.declare_parameter("act_exp1_action_dim", 13)
            .get_parameter_value()
            .integer_value
        )
        touch_dim = (
            node.declare_parameter("act_exp1_touch_dim", 10)
            .get_parameter_value()
            .integer_value
        )
        arm_reference_frame = (
            node.declare_parameter("arm_reference_frame", "fr3_link0")
            .get_parameter_value()
            .string_value
        )
        arm_duration_sec = (
            node.declare_parameter("arm_duration_sec", 0.1)
            .get_parameter_value()
            .double_value
        )
        hand_duration_sec = (
            node.declare_parameter("hand_duration_sec", 0.1)
            .get_parameter_value()
            .double_value
        )
        control_hz = (
            node.declare_parameter("act_exp1_control_hz", 10.0)
            .get_parameter_value()
            .double_value
        )
        fid = list(
            node.declare_parameter(
                "act_exp1_hand_finger_id_order", [0, 1, 2, 3, 4, 5]
            )
            .get_parameter_value()
            .integer_array_value
        )
        if len(fid) != 6:
            raise RuntimeError(
                f"act_exp1_hand_finger_id_order must have length 6, got {len(fid)}"
            )
        hand_finger_id_order = (
            int(fid[0]),
            int(fid[1]),
            int(fid[2]),
            int(fid[3]),
            int(fid[4]),
            int(fid[5]),
        )
        obs_camera_id = (
            node.declare_parameter("act_exp1_camera_id", "")
            .get_parameter_value()
            .string_value
        )
        obs_depth_camera_id = (
            node.declare_parameter("act_exp1_depth_camera_id", "")
            .get_parameter_value()
            .string_value
        )

        config = ACTExp1PolicyConfig(
            checkpoint_dir=checkpoint_dir,
            device=device,
            image_key=image_key,
            depth_key=depth_key,
            use_depth=bool(use_depth),
            state_key=state_key,
            img_h=int(img_h),
            img_w=int(img_w),
            state_dim=int(state_dim),
            action_dim=int(action_dim),
            touch_dim=int(touch_dim),
            arm_reference_frame=arm_reference_frame,
            arm_duration_sec=float(arm_duration_sec),
            hand_duration_sec=float(hand_duration_sec),
            control_hz=float(control_hz),
            hand_finger_id_order=hand_finger_id_order,
            obs_camera_id=str(obs_camera_id).strip(),
            obs_depth_camera_id=str(obs_depth_camera_id).strip(),
        )
        adapter = cls(node=node, config=config)
        node.get_logger().info(
            f"ACT exp1 policy loaded: checkpoint={checkpoint_dir} "
            f"device={adapter.torch_device} rgb_key={image_key} depth_key={depth_key} "
            f"use_depth={config.use_depth} state_key={state_key} "
            f"camera_id={config.obs_camera_id!r} depth_camera_id={config.obs_depth_camera_id!r}"
        )
        return adapter

    def _resolve_torch_device(self, requested: str) -> str:
        if requested == "auto":
            return "cuda" if torch.cuda.is_available() else "cpu"
        if requested not in ("cpu", "cuda"):
            raise RuntimeError(
                f"Invalid act_exp1_device={requested}, expected auto|cpu|cuda"
            )
        if requested == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("act_exp1_device=cuda but CUDA is not available")
        return requested

    def _load_policy(self, checkpoint_dir: str) -> Any:
        try:
            from lerobot.policies.act.modeling_act import ACTPolicy
        except Exception as exc:
            raise RuntimeError("Failed to import ACTPolicy from lerobot") from exc

        if not checkpoint_dir:
            raise RuntimeError("act_exp1_checkpoint_dir is empty while policy_type=act_exp1")
        ckpt = Path(checkpoint_dir)
        if not ckpt.exists():
            raise RuntimeError(f"ACT exp1 checkpoint dir does not exist: {ckpt}")

        try:
            policy = ACTPolicy.from_pretrained(str(ckpt))
            policy.to(self._torch_device)
            policy.eval()
            return policy
        except Exception as exc:
            raise RuntimeError(f"Failed to load ACT exp1 policy from {ckpt}") from exc

    def _rgb_to_numpy(self, obs: RobotObservation) -> np.ndarray:
        image, _, _ = resolve_observation_images(
            obs,
            self._config.obs_camera_id,
            self._config.obs_depth_camera_id,
            self._node,
            "_warned_act_exp1_obs_camera",
            self,
        )
        if image.height <= 0 or image.width <= 0:
            raise ValueError("rgb_image has invalid width/height")
        channels = 3
        expected_min_step = int(image.width) * channels
        if image.step < expected_min_step:
            raise ValueError(
                f"rgb_image step too small: {image.step}, expected >= {expected_min_step}"
            )
        raw = np.frombuffer(image.data, dtype=np.uint8)
        if raw.size != int(image.height) * int(image.step):
            raise ValueError(
                f"rgb_image bytes mismatch: got {raw.size}, "
                f"expected {int(image.height) * int(image.step)}"
            )
        img = raw.reshape((int(image.height), int(image.step)))[:, : expected_min_step]
        img = img.reshape((int(image.height), int(image.width), channels))
        if image.encoding.lower() == "bgr8":
            img = img[..., ::-1]
        return np.ascontiguousarray(img)

    def _depth_to_numpy_hwc3(self, image: Image) -> np.ndarray:
        """Build HxWx3 float32 in [0, 1] for ``observation.rs_depth`` (matches 3-ch video in info.json)."""
        h, w = int(image.height), int(image.width)
        if h <= 0 or w <= 0:
            raise ValueError("depth_image has invalid width/height")
        enc = image.encoding.lower()
        step = int(image.step)
        raw_u8 = np.frombuffer(image.data, dtype=np.uint8)
        if raw_u8.size != h * step:
            raise ValueError(
                f"depth_image bytes mismatch: got {raw_u8.size}, expected {h * step}"
            )
        row = raw_u8.reshape(h, step)

        if enc in ("16uc1", "mono16"):
            if step < w * 2:
                raise ValueError(f"depth 16UC1 step too small: {step}")
            arr_u16 = row[:, : w * 2].copy().view(np.uint16).reshape(h, w)
            depth_m = arr_u16.astype(np.float32) / 1000.0
        elif enc in ("32fc1",):
            if step < w * 4:
                raise ValueError(f"depth 32FC1 step too small: {step}")
            depth_m = row[:, : w * 4].copy().view(np.float32).reshape(h, w)
        elif enc in ("8uc3", "rgb8", "bgr8"):
            c = 3
            if step < w * c:
                raise ValueError(f"depth 3-channel step too small: {step}")
            img = row[:, : w * c].reshape(h, w, c).astype(np.float32) / 255.0
            if enc == "bgr8":
                img = img[..., ::-1]
            return np.ascontiguousarray(img)
        else:
            raise ValueError(
                f"Unsupported depth_image encoding={image.encoding}; "
                "use 16UC1, 32FC1, or 8UC3/RGB8/BGR8"
            )

        valid = depth_m > 0
        norm = np.zeros((h, w), dtype=np.float32)
        if np.any(valid):
            vmin = float(np.min(depth_m[valid]))
            vmax = float(np.max(depth_m[valid]))
            if vmax > vmin:
                norm[valid] = (depth_m[valid] - vmin) / (vmax - vmin)
        hwc = np.stack([norm, norm, norm], axis=-1)
        return np.ascontiguousarray(hwc)

    def _build_touch_forces(self, obs: RobotObservation) -> np.ndarray:
        n = np.asarray(obs.hand_touch.normal_forces, dtype=np.float32).reshape(-1)
        t = np.asarray(obs.hand_touch.tangential_forces, dtype=np.float32).reshape(-1)
        combined = np.concatenate([n, t])
        dim = self._config.touch_dim
        out = np.zeros(dim, dtype=np.float32)
        ncopy = min(dim, combined.size)
        out[:ncopy] = combined[:ncopy]
        if combined.size != dim and not self._warned_touch_shape:
            self._node.get_logger().warn(
                f"Touch concat length {combined.size} != act_exp1_touch_dim={dim}; "
                f"padded or truncated to match training."
            )
            self._warned_touch_shape = True
        return out

    def _build_state(self, obs: RobotObservation) -> np.ndarray:
        if len(obs.arm_joint_position) != 7:
            raise ValueError(
                f"arm_joint_position size mismatch: got {len(obs.arm_joint_position)}, expected 7"
            )
        if len(obs.hand_joint_position) != 6:
            raise ValueError(
                f"hand_joint_position size mismatch: got {len(obs.hand_joint_position)}, expected 6"
            )

        state = np.zeros((self._config.state_dim,), dtype=np.float32)
        state[0:7] = np.asarray(obs.arm_joint_position, dtype=np.float32)
        state[7] = float(obs.ee_pose.position.x)
        state[8] = float(obs.ee_pose.position.y)
        state[9] = float(obs.ee_pose.position.z)
        state[10] = float(obs.ee_pose.orientation.x)
        state[11] = float(obs.ee_pose.orientation.y)
        state[12] = float(obs.ee_pose.orientation.z)
        state[13] = float(obs.ee_pose.orientation.w)
        state[14:20] = np.asarray(obs.hand_joint_position, dtype=np.float32)
        state[20 : 20 + self._config.touch_dim] = self._build_touch_forces(obs)

        if state.shape[0] != self._config.state_dim:
            raise ValueError(
                f"Built state length {state.shape[0]} != act_exp1_state_dim={self._config.state_dim}"
            )
        return state

    def _build_batch(self, obs: RobotObservation) -> Dict[str, torch.Tensor]:
        rgb = self._rgb_to_numpy(obs)
        state = self._build_state(obs)
        rgb_t = torch.from_numpy(rgb).permute(2, 0, 1).unsqueeze(0).float() / 255.0
        rgb_t = F.interpolate(
            rgb_t,
            size=(self._config.img_h, self._config.img_w),
            mode="area",
        )
        state_t = torch.from_numpy(state).unsqueeze(0)
        out: Dict[str, torch.Tensor] = {
            self._config.image_key: rgb_t.to(self._torch_device),
            self._config.state_key: state_t.to(self._torch_device),
        }
        if self._config.use_depth:
            _, depth_img, _ = resolve_observation_images(
                obs,
                self._config.obs_camera_id,
                self._config.obs_depth_camera_id,
                self._node,
                "_warned_act_exp1_obs_camera",
                self,
            )
            if depth_img.height <= 0 or depth_img.width <= 0:
                raise ValueError("depth_image missing or empty; required for act_exp1_use_depth=true")
            depth_hwc = self._depth_to_numpy_hwc3(depth_img)
            depth_t = torch.from_numpy(depth_hwc).permute(2, 0, 1).unsqueeze(0).float()
            depth_t = F.interpolate(
                depth_t,
                size=(self._config.img_h, self._config.img_w),
                mode="area",
            )
            out[self._config.depth_key] = depth_t.to(self._torch_device)
        return out

    def _decode_action(self, action_out: Any) -> np.ndarray:
        if isinstance(action_out, dict):
            if "action" in action_out:
                action_out = action_out["action"]
            elif len(action_out) > 0:
                action_out = next(iter(action_out.values()))
            else:
                raise ValueError("ACT output dict is empty")
        if not isinstance(action_out, torch.Tensor):
            raise TypeError(f"ACT output is not Tensor: {type(action_out)}")
        action = action_out.detach().float().cpu()
        if action.ndim == 2:
            action = action[0]
        elif action.ndim == 3:
            action = action[0, 0]
        else:
            raise ValueError(f"Unsupported ACT output shape: {tuple(action.shape)}")
        action_np = action.numpy()
        if action_np.shape != (self._config.action_dim,):
            raise ValueError(
                f"ACT action shape mismatch: got {action_np.shape}, "
                f"expected ({self._config.action_dim},)"
            )
        return action_np

    def _to_whole_body_action(self, action: np.ndarray) -> WholeBodyAction:
        if action.shape != (13,):
            raise ValueError(f"Expected action shape (13,), got {action.shape}")
        msg = WholeBodyAction()
        msg.header.stamp = self._node.get_clock().now().to_msg()

        msg.arm.control_mode = 0
        msg.arm.is_relative = False
        msg.arm.reference_frame = self._config.arm_reference_frame
        msg.arm.joint_position = [float(x) for x in action[0:7]]
        msg.arm.joint_velocity = []
        msg.arm.duration_sec = float(self._config.arm_duration_sec)

        msg.hand.control_mode = 0
        msg.hand.is_relative = False
        hand_rad = action[7:13]
        hand_norm = [
            _normalized_position_from_radians(
                float(hand_rad[i]), self._config.hand_finger_id_order[i]
            )
            for i in range(6)
        ]
        msg.hand.joint_position = hand_norm
        msg.hand.joint_velocity = []
        msg.hand.duration_sec = float(self._config.hand_duration_sec)
        return msg

    def infer(self, obs: RobotObservation) -> Optional[WholeBodyAction]:
        batch = self._build_batch(obs)
        with torch.inference_mode():
            action_out = self._policy.select_action(batch)
        action = self._decode_action(action_out)
        return self._to_whole_body_action(action)
