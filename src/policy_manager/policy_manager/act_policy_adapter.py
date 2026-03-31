from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
import torch
import torch.nn.functional as F
from rclpy.node import Node
from robot_interfaces.msg import RobotObservation, WholeBodyAction

from policy_manager.base_policy import BasePolicy


@dataclass(frozen=True)
class ACTPolicyConfig:
    checkpoint_dir: str
    device: str
    image_key: str
    state_key: str
    img_h: int
    img_w: int
    state_dim: int
    action_dim: int
    arm_reference_frame: str
    arm_duration_sec: float
    hand_duration_sec: float
    control_hz: float


class ACTPolicyAdapter(BasePolicy):
    def __init__(self, node: Node, config: ACTPolicyConfig) -> None:
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
        checkpoint_dir = (
            node.declare_parameter("act_checkpoint_dir", "")
            .get_parameter_value()
            .string_value
        )
        device = (
            node.declare_parameter("act_device", "auto")
            .get_parameter_value()
            .string_value
        )
        image_key = (
            node.declare_parameter("act_image_key", "observation.camera_3.rgb")
            .get_parameter_value()
            .string_value
        )
        state_key = (
            node.declare_parameter("act_state_key", "observation.state")
            .get_parameter_value()
            .string_value
        )
        img_h = (
            node.declare_parameter("act_img_h", 256)
            .get_parameter_value()
            .integer_value
        )
        img_w = (
            node.declare_parameter("act_img_w", 256)
            .get_parameter_value()
            .integer_value
        )
        state_dim = (
            node.declare_parameter("act_state_dim", 13)
            .get_parameter_value()
            .integer_value
        )
        action_dim = (
            node.declare_parameter("act_action_dim", 13)
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
            node.declare_parameter("act_control_hz", 3.0)
            .get_parameter_value()
            .double_value
        )

        config = ACTPolicyConfig(
            checkpoint_dir=checkpoint_dir,
            device=device,
            image_key=image_key,
            state_key=state_key,
            img_h=int(img_h),
            img_w=int(img_w),
            state_dim=int(state_dim),
            action_dim=int(action_dim),
            arm_reference_frame=arm_reference_frame,
            arm_duration_sec=float(arm_duration_sec),
            hand_duration_sec=float(hand_duration_sec),
            control_hz=float(control_hz),
        )
        adapter = cls(node=node, config=config)
        node.get_logger().info(
            f"ACT policy loaded: checkpoint={checkpoint_dir} "
            f"device={adapter.torch_device} image_key={image_key} state_key={state_key}"
        )
        return adapter

    def _resolve_torch_device(self, requested: str) -> str:
        if requested == 'auto':
            return 'cuda' if torch.cuda.is_available() else 'cpu'
        if requested not in ('cpu', 'cuda'):
            raise RuntimeError(f'Invalid act_device={requested}, expected auto|cpu|cuda')
        if requested == 'cuda' and not torch.cuda.is_available():
            raise RuntimeError('act_device=cuda but CUDA is not available')
        return requested

    def _load_policy(self, checkpoint_dir: str) -> Any:
        try:
            from lerobot.policies.act.modeling_act import ACTPolicy
        except Exception as exc:
            raise RuntimeError('Failed to import ACTPolicy from lerobot') from exc

        if not checkpoint_dir:
            raise RuntimeError('act_checkpoint_dir is empty while policy_type=act')
        ckpt = Path(checkpoint_dir)
        if not ckpt.exists():
            raise RuntimeError(f'ACT checkpoint dir does not exist: {ckpt}')

        try:
            policy = ACTPolicy.from_pretrained(str(ckpt))
            policy.to(self._torch_device)
            policy.eval()
            return policy
        except Exception as exc:
            raise RuntimeError(f'Failed to load ACT policy from {ckpt}') from exc

    def _rgb_to_numpy(self, obs: RobotObservation) -> np.ndarray:
        image = obs.rgb_image
        if image.height <= 0 or image.width <= 0:
            raise ValueError('rgb_image has invalid width/height')
        channels = 3
        expected_min_step = int(image.width) * channels
        if image.step < expected_min_step:
            raise ValueError(
                f'rgb_image step too small: {image.step}, expected >= {expected_min_step}'
            )
        raw = np.frombuffer(image.data, dtype=np.uint8)
        if raw.size != int(image.height) * int(image.step):
            raise ValueError(
                f'rgb_image bytes mismatch: got {raw.size}, '
                f'expected {int(image.height) * int(image.step)}'
            )
        img = raw.reshape((int(image.height), int(image.step)))[:, : expected_min_step]
        img = img.reshape((int(image.height), int(image.width), channels))
        if image.encoding.lower() == 'bgr8':
            img = img[..., ::-1]
        return np.ascontiguousarray(img)

    def _build_state(self, obs: RobotObservation) -> np.ndarray:
        if len(obs.hand_joint_position) != 6:
            raise ValueError(
                f'hand_joint_position size mismatch: got {len(obs.hand_joint_position)}, expected 6'
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
                f'act_state_dim must be 13 for current mapping, got {self._config.state_dim}'
            )
        return state

    def _build_batch(self, obs: RobotObservation) -> Dict[str, torch.Tensor]:
        rgb = self._rgb_to_numpy(obs)
        state = self._build_state(obs)
        rgb_t = torch.from_numpy(rgb).permute(2, 0, 1).unsqueeze(0).float() / 255.0
        # 统一用 area 模式缩放到模型输入尺寸。
        rgb_t = F.interpolate(
            rgb_t,
            size=(self._config.img_h, self._config.img_w),
            mode="area",
        )
        state_t = torch.from_numpy(state).unsqueeze(0)
        return {
            self._config.image_key: rgb_t.to(self._torch_device),
            self._config.state_key: state_t.to(self._torch_device),
        }

    def _decode_action(self, action_out: Any) -> np.ndarray:
        if isinstance(action_out, dict):
            if 'action' in action_out:
                action_out = action_out['action']
            elif len(action_out) > 0:
                action_out = next(iter(action_out.values()))
            else:
                raise ValueError('ACT output dict is empty')
        if not isinstance(action_out, torch.Tensor):
            raise TypeError(f'ACT output is not Tensor: {type(action_out)}')
        action = action_out.detach().float().cpu()
        if action.ndim == 2:
            action = action[0]
        elif action.ndim == 3:
            action = action[0, 0]
        else:
            raise ValueError(f'Unsupported ACT output shape: {tuple(action.shape)}')
        action_np = action.numpy()
        if action_np.shape != (self._config.action_dim,):
            raise ValueError(
                f'ACT action shape mismatch: got {action_np.shape}, '
                f'expected ({self._config.action_dim},)'
            )
        return action_np

    def _to_whole_body_action(self, action: np.ndarray) -> WholeBodyAction:
        if action.shape != (13,):
            raise ValueError(f'Expected action shape (13,), got {action.shape}')
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
        msg.hand.joint_position = [float(x) for x in action[7:13]]
        msg.hand.duration_sec = float(self._config.hand_duration_sec)
        return msg

    def infer(self, obs: RobotObservation) -> Optional[WholeBodyAction]:
        batch = self._build_batch(obs)
        with torch.inference_mode():
            action_out = self._policy.select_action(batch)
        action = self._decode_action(action_out)
        return self._to_whole_body_action(action)
