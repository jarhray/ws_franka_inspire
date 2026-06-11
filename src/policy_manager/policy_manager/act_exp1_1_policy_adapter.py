"""ACT adapter for dataset layout in data_recorded/exp1_1/meta/info.json (13-dim state / action)."""

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

from policy_manager.act_exp1_policy_adapter import _normalized_position_from_radians
from policy_manager.base_policy import BasePolicy
from policy_manager.observation_camera import resolve_observation_images


@dataclass(frozen=True)
class ACTExp11PolicyConfig:
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
    arm_reference_frame: str
    arm_duration_sec: float
    hand_duration_sec: float
    control_hz: float
    hand_finger_id_order: tuple[int, int, int, int, int, int]
    obs_camera_id: str = ""
    obs_depth_camera_id: str = ""


class ACTExp11PolicyAdapter(BasePolicy):
    """Matches exp1_1 / info.json: 13-dim state and action.

    observation.state (13): ee_pose (x,y,z,qx,qy,qz,qw) + hand_angles (6).
    observation.images.rs_color + observation.rs_depth (optional, 3×H×W).
    action (13): same semantics -> arm cartesian_pose + hand joint targets (rad -> [-1,1] for Inspire).
    """

    def __init__(self, node: Node, config: ACTExp11PolicyConfig) -> None:
        self._node = node
        self._config = config
        self._torch_device = self._resolve_torch_device(config.device)
        self._policy = self._load_policy(config.checkpoint_dir)

    @property
    def torch_device(self) -> str:
        return self._torch_device

    @property
    def control_hz(self) -> float:
        return float(self._config.control_hz)

    @classmethod
    def from_node(cls, node: Node) -> "BasePolicy":
        p = "act_exp1_1"
        checkpoint_dir = (
            node.declare_parameter(f"{p}_checkpoint_dir", "")
            .get_parameter_value()
            .string_value
        )
        device = (
            node.declare_parameter(f"{p}_device", "auto")
            .get_parameter_value()
            .string_value
        )
        image_key = (
            node.declare_parameter(
                f"{p}_image_key", "observation.images.rs_color"
            )
            .get_parameter_value()
            .string_value
        )
        depth_key = (
            node.declare_parameter(f"{p}_depth_key", "observation.rs_depth")
            .get_parameter_value()
            .string_value
        )
        use_depth = (
            node.declare_parameter(f"{p}_use_depth", True)
            .get_parameter_value()
            .bool_value
        )
        state_key = (
            node.declare_parameter(f"{p}_state_key", "observation.state")
            .get_parameter_value()
            .string_value
        )
        img_h = (
            node.declare_parameter(f"{p}_img_h", 480)
            .get_parameter_value()
            .integer_value
        )
        img_w = (
            node.declare_parameter(f"{p}_img_w", 640)
            .get_parameter_value()
            .integer_value
        )
        state_dim = (
            node.declare_parameter(f"{p}_state_dim", 13)
            .get_parameter_value()
            .integer_value
        )
        action_dim = (
            node.declare_parameter(f"{p}_action_dim", 13)
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
            node.declare_parameter(f"{p}_control_hz", 10.0)
            .get_parameter_value()
            .double_value
        )
        fid = list(
            node.declare_parameter(
                f"{p}_hand_finger_id_order", [0, 1, 2, 3, 4, 5]
            )
            .get_parameter_value()
            .integer_array_value
        )
        if len(fid) != 6:
            raise RuntimeError(
                f"{p}_hand_finger_id_order must have length 6, got {len(fid)}"
            )
        hand_finger_id_order = tuple(int(x) for x in fid)
        obs_camera_id = (
            node.declare_parameter(f"{p}_camera_id", "")
            .get_parameter_value()
            .string_value
        )
        obs_depth_camera_id = (
            node.declare_parameter(f"{p}_depth_camera_id", "")
            .get_parameter_value()
            .string_value
        )

        config = ACTExp11PolicyConfig(
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
            arm_reference_frame=arm_reference_frame,
            arm_duration_sec=float(arm_duration_sec),
            hand_duration_sec=float(hand_duration_sec),
            control_hz=float(control_hz),
            hand_finger_id_order=(
                hand_finger_id_order[0],
                hand_finger_id_order[1],
                hand_finger_id_order[2],
                hand_finger_id_order[3],
                hand_finger_id_order[4],
                hand_finger_id_order[5],
            ),
            obs_camera_id=str(obs_camera_id).strip(),
            obs_depth_camera_id=str(obs_depth_camera_id).strip(),
        )
        adapter = cls(node=node, config=config)
        node.get_logger().info(
            f"ACT exp1_1 policy loaded: checkpoint={checkpoint_dir} "
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
                f"Invalid act_exp1_1_device={requested}, expected auto|cpu|cuda"
            )
        if requested == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("act_exp1_1_device=cuda but CUDA is not available")
        return requested

    def _load_policy(self, checkpoint_dir: str) -> Any:
        try:
            from lerobot.policies.act.modeling_act import ACTPolicy
        except Exception as exc:
            raise RuntimeError("Failed to import ACTPolicy from lerobot") from exc

        if not checkpoint_dir:
            raise RuntimeError(
                "act_exp1_1_checkpoint_dir is empty while policy_type=act_exp1_1"
            )
        ckpt = Path(checkpoint_dir)
        if not ckpt.exists():
            raise RuntimeError(f"ACT exp1_1 checkpoint dir does not exist: {ckpt}")

        try:
            policy = ACTPolicy.from_pretrained(str(ckpt))
            policy.to(self._torch_device)
            policy.eval()
            return policy
        except Exception as exc:
            raise RuntimeError(f"Failed to load ACT exp1_1 policy from {ckpt}") from exc

    def _rgb_to_numpy(self, obs: RobotObservation) -> np.ndarray:
        image, _, _ = resolve_observation_images(
            obs,
            self._config.obs_camera_id,
            self._config.obs_depth_camera_id,
            self._node,
            "_warned_act_exp11_obs_camera",
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

    def _build_state(self, obs: RobotObservation) -> np.ndarray:
        if len(obs.hand_joint_position) != 6:
            raise ValueError(
                f"hand_joint_position size mismatch: got {len(obs.hand_joint_position)}, expected 6"
            )
        state = np.zeros((13,), dtype=np.float32)
        state[0] = float(obs.ee_pose.position.x)
        state[1] = float(obs.ee_pose.position.y)
        state[2] = float(obs.ee_pose.position.z)
        state[3] = float(obs.ee_pose.orientation.x)
        state[4] = float(obs.ee_pose.orientation.y)
        state[5] = float(obs.ee_pose.orientation.z)
        state[6] = float(obs.ee_pose.orientation.w)
        state[7:13] = np.asarray(obs.hand_joint_position, dtype=np.float32)
        if self._config.state_dim != 13:
            raise ValueError(
                f"act_exp1_1_state_dim must be 13 for this layout, got {self._config.state_dim}"
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
                "_warned_act_exp11_obs_camera",
                self,
            )
            if depth_img.height <= 0 or depth_img.width <= 0:
                raise ValueError(
                    "depth_image missing or empty; required for act_exp1_1_use_depth=true"
                )
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

        msg.arm.control_mode = 2
        msg.arm.is_relative = False
        msg.arm.reference_frame = self._config.arm_reference_frame
        msg.arm.cartesian_pose.position.x = float(action[0])
        msg.arm.cartesian_pose.position.y = float(action[1])
        msg.arm.cartesian_pose.position.z = float(action[2])
        msg.arm.cartesian_pose.orientation.x = float(action[3])
        msg.arm.cartesian_pose.orientation.y = float(action[4])
        msg.arm.cartesian_pose.orientation.z = float(action[5])
        msg.arm.cartesian_pose.orientation.w = float(action[6])
        msg.arm.duration_sec = float(self._config.arm_duration_sec)

        msg.hand.control_mode = 0
        msg.hand.is_relative = False
        hand_rad = action[7:13]
        msg.hand.joint_position = [
            _normalized_position_from_radians(
                float(hand_rad[i]), self._config.hand_finger_id_order[i]
            )
            for i in range(6)
        ]
        msg.hand.joint_velocity = []
        msg.hand.duration_sec = float(self._config.hand_duration_sec)
        return msg

    def infer(self, obs: RobotObservation) -> Optional[WholeBodyAction]:
        batch = self._build_batch(obs)
        with torch.inference_mode():
            action_out = self._policy.select_action(batch)
        action = self._decode_action(action_out)
        return self._to_whole_body_action(action)
