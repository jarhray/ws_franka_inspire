"""ACT adapter for exp3_1 layout (multi-camera RGB/depth + 8-dim joint state/action)."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

import cv2
import numpy as np
import torch
from rclpy.node import Node
from sensor_msgs.msg import Image

from robot_interfaces.msg import RobotObservation, WholeBodyAction

from policy_manager.base_policy import BasePolicy
from policy_manager.observation_camera import resolve_observation_images


@dataclass(frozen=True)
class ACTExp31PolicyConfig:
    checkpoint_dir: str
    device: str
    image_keys: tuple[str, ...]
    depth_keys: tuple[str, ...]
    camera_ids: tuple[str, ...]
    depth_camera_ids: tuple[str, ...]
    use_depth: bool
    state_key: str
    img_h: int
    img_w: int
    depth_clip_mm_min: float
    depth_clip_mm_max: float
    state_dim: int
    action_dim: int
    arm_duration_sec: float
    hand_duration_sec: float
    control_hz: float
    hand_finger_id_order: tuple[int, int, int, int, int, int]
    hand_binary_threshold: float
    hand_binary_reset_hardware: tuple[float, float, float, float, float, float]
    hand_binary_grasp_hardware: tuple[float, float, float, float, float, float]


class ACTExp31PolicyAdapter(BasePolicy):
    """Matches exp3_1 info.json: state/action=(7 arm joints + hand_grasp_binary), multi-camera RGB/depth."""

    def __init__(self, node: Node, config: ACTExp31PolicyConfig) -> None:
        self._node = node
        self._config = config
        self._torch_device = self._resolve_torch_device(config.device)
        self._policy, self._preprocessor, self._postprocessor = self._load_policy_bundle(
            config.checkpoint_dir
        )

    @property
    def control_hz(self) -> float:
        return float(self._config.control_hz)

    @classmethod
    def from_node(cls, node: Node) -> "BasePolicy":
        p = "act_exp3_1"
        checkpoint_dir = node.declare_parameter(f"{p}_checkpoint_dir", "").value
        device = node.declare_parameter(f"{p}_device", "auto").value
        image_keys = list(
            node.declare_parameter(
                f"{p}_image_keys",
                [
                    "observation.images.cam1_color",
                    "observation.images.cam3_color",
                    "observation.images.cam4_color",
                ],
            ).value
            or []
        )
        depth_keys = list(
            node.declare_parameter(
                f"{p}_depth_keys",
                [
                    "observation.cam1_depth",
                    "observation.cam3_depth",
                    "observation.cam4_depth",
                ],
            ).value
            or []
        )
        camera_ids = list(
            node.declare_parameter(f"{p}_camera_ids", ["cam1", "cam3", "cam4"]).value or []
        )
        depth_camera_ids = list(
            node.declare_parameter(f"{p}_depth_camera_ids", ["cam1", "cam3", "cam4"]).value
            or []
        )
        use_depth = bool(node.declare_parameter(f"{p}_use_depth", True).value)
        state_key = node.declare_parameter(f"{p}_state_key", "observation.state").value
        img_h = int(node.declare_parameter(f"{p}_img_h", 480).value)
        img_w = int(node.declare_parameter(f"{p}_img_w", 640).value)
        depth_clip_mm_min = float(node.declare_parameter(f"{p}_depth_clip_mm_min", 100.0).value)
        depth_clip_mm_max = float(node.declare_parameter(f"{p}_depth_clip_mm_max", 8000.0).value)
        state_dim = int(node.declare_parameter(f"{p}_state_dim", 8).value)
        action_dim = int(node.declare_parameter(f"{p}_action_dim", 8).value)
        control_hz = float(node.declare_parameter(f"{p}_control_hz", 10.0).value)
        arm_duration_sec = float(node.declare_parameter("arm_duration_sec", 0.1).value)
        hand_duration_sec = float(node.declare_parameter("hand_duration_sec", 0.1).value)
        fid = list(node.declare_parameter(f"{p}_hand_finger_id_order", [0, 1, 2, 3, 4, 5]).value)
        if len(fid) != 6:
            raise RuntimeError(f"{p}_hand_finger_id_order must have length 6, got {len(fid)}")
        threshold = float(node.declare_parameter(f"{p}_hand_binary_threshold", 0.5).value)
        reset = list(
            node.declare_parameter(
                f"{p}_hand_binary_reset_hardware", [1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 0.0]
            ).value
        )
        grasp = list(
            node.declare_parameter(
                f"{p}_hand_binary_grasp_hardware", [500.0, 500.0, 500.0, 500.0, 500.0, 0.0]
            ).value
        )
        if len(reset) != 6 or len(grasp) != 6:
            raise RuntimeError(f"{p}_hand_binary_(reset|grasp)_hardware must be length 6")
        if len(image_keys) == 0:
            raise RuntimeError(f"{p}_image_keys must be non-empty")
        if len(camera_ids) != len(image_keys):
            raise RuntimeError(
                f"{p}_camera_ids length ({len(camera_ids)}) must match {p}_image_keys length ({len(image_keys)})"
            )
        if use_depth:
            if len(depth_keys) != len(image_keys):
                raise RuntimeError(
                    f"{p}_depth_keys length ({len(depth_keys)}) must match {p}_image_keys length ({len(image_keys)})"
                )
            if len(depth_camera_ids) != len(image_keys):
                raise RuntimeError(
                    f"{p}_depth_camera_ids length ({len(depth_camera_ids)}) must match {p}_image_keys length ({len(image_keys)})"
                )
        else:
            depth_keys = []
            depth_camera_ids = []

        config = ACTExp31PolicyConfig(
            checkpoint_dir=str(checkpoint_dir),
            device=str(device),
            image_keys=tuple(str(x) for x in image_keys),
            depth_keys=tuple(str(x) for x in depth_keys),
            camera_ids=tuple(str(x).strip() for x in camera_ids),
            depth_camera_ids=tuple(str(x).strip() for x in depth_camera_ids),
            use_depth=bool(use_depth),
            state_key=str(state_key),
            img_h=img_h,
            img_w=img_w,
            depth_clip_mm_min=depth_clip_mm_min,
            depth_clip_mm_max=depth_clip_mm_max,
            state_dim=state_dim,
            action_dim=action_dim,
            arm_duration_sec=arm_duration_sec,
            hand_duration_sec=hand_duration_sec,
            control_hz=control_hz,
            hand_finger_id_order=tuple(int(x) for x in fid),
            hand_binary_threshold=threshold,
            hand_binary_reset_hardware=tuple(float(x) for x in reset),
            hand_binary_grasp_hardware=tuple(float(x) for x in grasp),
        )
        adapter = cls(node=node, config=config)
        node.get_logger().info(
            f"ACT exp3_1 policy loaded: checkpoint={checkpoint_dir} device={adapter._torch_device} "
            f"camera_ids={list(config.camera_ids)} use_depth={config.use_depth}"
        )
        return adapter

    @staticmethod
    def _radians_from_hardware_four_fingers(k: float) -> float:
        return -5e-10 * k**3 + 9e-7 * k**2 - 0.0018 * k + 1.4191

    @staticmethod
    def _radians_from_hardware_thumb_flexion(k: float) -> float:
        return 8e-11 * k**3 - 4e-8 * k**2 - 0.0006 * k + 0.5869

    @staticmethod
    def _radians_from_hardware_thumb_abduction(k: float) -> float:
        return -0.0012 * k + 1.1641

    @classmethod
    def _radians_from_hardware_for_finger_id(cls, k: float, finger_id: int) -> float:
        if finger_id in (0, 1, 2, 3):
            return cls._radians_from_hardware_four_fingers(k)
        if finger_id == 4:
            return cls._radians_from_hardware_thumb_flexion(k)
        if finger_id == 5:
            return cls._radians_from_hardware_thumb_abduction(k)
        return cls._radians_from_hardware_four_fingers(k)

    @staticmethod
    def _normalized_from_hardware(k: float) -> float:
        return float(np.clip((k / 500.0) - 1.0, -1.0, 1.0))

    def _resolve_torch_device(self, requested: str) -> str:
        if requested == "auto":
            return "cuda" if torch.cuda.is_available() else "cpu"
        if requested not in ("cpu", "cuda"):
            raise RuntimeError(f"Invalid act_exp3_1_device={requested}, expected auto|cpu|cuda")
        if requested == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("act_exp3_1_device=cuda but CUDA is not available")
        return requested

    def _load_policy_bundle(self, checkpoint_dir: str) -> tuple[Any, Any, Any]:
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies.act.modeling_act import ACTPolicy
        from lerobot.policies.factory import make_pre_post_processors

        if not checkpoint_dir:
            raise RuntimeError("act_exp3_1_checkpoint_dir is empty while policy_type=act_exp3_1")
        ckpt = Path(checkpoint_dir)
        if not ckpt.exists():
            raise RuntimeError(f"ACT exp3_1 checkpoint dir does not exist: {ckpt}")
        policy_cfg = PreTrainedConfig.from_pretrained(str(ckpt))
        policy_cfg.device = self._torch_device
        policy = ACTPolicy.from_pretrained(str(ckpt), config=policy_cfg)
        policy.to(self._torch_device)
        policy.eval()
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=policy_cfg,
            pretrained_path=str(ckpt),
            preprocessor_overrides={
                "device_processor": {"device": self._torch_device},
            },
        )
        return policy, preprocessor, postprocessor

    def _rgb_to_numpy(self, image: Image) -> np.ndarray:
        if image.height <= 0 or image.width <= 0:
            raise ValueError("rgb_image has invalid width/height")
        h, w = int(image.height), int(image.width)
        enc = (image.encoding or "").lower()
        raw = np.frombuffer(image.data, dtype=np.uint8)
        step = int(image.step)
        if raw.size != h * step:
            raise ValueError("rgb_image bytes mismatch")

        if enc in ("rgb8", "bgr8"):
            if step < w * 3:
                raise ValueError(f"rgb_image step too small: {step}, expected >= {w * 3}")
            arr = raw.reshape(h, step)[:, : w * 3].reshape(h, w, 3)
            if enc == "bgr8":
                arr = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
        elif enc == "mono8":
            if step < w:
                raise ValueError(f"mono8 rgb_image step too small: {step}, expected >= {w}")
            gray = raw.reshape(h, step)[:, :w].reshape(h, w)
            arr = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
        else:
            raise ValueError(f"Unsupported rgb_image encoding: {image.encoding}")

        arr = cv2.resize(arr, (self._config.img_w, self._config.img_h), interpolation=cv2.INTER_LINEAR)
        return np.ascontiguousarray(arr)

    @staticmethod
    def _depth_to_rgb_u8(depth_mm: np.ndarray, clip_mm: tuple[float, float]) -> np.ndarray:
        lo, hi = clip_mm
        d = np.clip(depth_mm, lo, hi)
        dn = (d - lo) / (hi - lo + 1e-9)
        u8 = (dn * 255.0).astype(np.uint8)
        return np.stack([u8, u8, u8], axis=-1)

    def _depth_to_numpy_hwc3(self, image: Image) -> np.ndarray:
        h, w = int(image.height), int(image.width)
        if h <= 0 or w <= 0:
            raise ValueError("depth_image has invalid width/height")
        enc = (image.encoding or "").lower()
        step = int(image.step)
        raw_u8 = np.frombuffer(image.data, dtype=np.uint8)
        if raw_u8.size != h * step:
            raise ValueError(f"depth_image bytes mismatch: got {raw_u8.size}, expected {h * step}")
        row = raw_u8.reshape(h, step)

        if enc in ("16uc1", "mono16"):
            if step < w * 2:
                raise ValueError(f"depth 16UC1 step too small: {step}")
            depth_mm = row[:, : w * 2].copy().view(np.uint16).reshape(h, w).astype(np.float32)
            depth_mm = cv2.resize(
                depth_mm, (self._config.img_w, self._config.img_h), interpolation=cv2.INTER_NEAREST
            )
            rgb = self._depth_to_rgb_u8(
                depth_mm, (self._config.depth_clip_mm_min, self._config.depth_clip_mm_max)
            )
            return np.ascontiguousarray(rgb)

        if enc == "32fc1":
            if step < w * 4:
                raise ValueError(f"depth 32FC1 step too small: {step}")
            depth_mm = row[:, : w * 4].copy().view(np.float32).reshape(h, w) * 1000.0
            depth_mm = cv2.resize(
                depth_mm, (self._config.img_w, self._config.img_h), interpolation=cv2.INTER_NEAREST
            )
            rgb = self._depth_to_rgb_u8(
                depth_mm, (self._config.depth_clip_mm_min, self._config.depth_clip_mm_max)
            )
            return np.ascontiguousarray(rgb)

        if enc in ("rgb8", "bgr8", "8uc3"):
            if step < w * 3:
                raise ValueError(f"depth rgb step too small: {step}")
            img = row[:, : w * 3].reshape(h, w, 3)
            if enc == "bgr8":
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img = cv2.resize(img, (self._config.img_w, self._config.img_h), interpolation=cv2.INTER_LINEAR)
            return np.ascontiguousarray(img)

        raise ValueError(
            f"Unsupported depth_image encoding={image.encoding}; use 16UC1, 32FC1 or RGB/BGR8"
        )

    def _build_obs_hand_binary(self, obs: RobotObservation) -> float:
        if len(obs.hand_joint_position) != 6:
            raise ValueError(f"hand_joint_position size mismatch: got {len(obs.hand_joint_position)}, expected 6")
        hand_rad = np.asarray(obs.hand_joint_position, dtype=np.float32)
        reset_rad = np.array(
            [
                self._radians_from_hardware_for_finger_id(
                    float(self._config.hand_binary_reset_hardware[i]),
                    self._config.hand_finger_id_order[i],
                )
                for i in range(6)
            ],
            dtype=np.float32,
        )
        grasp_rad = np.array(
            [
                self._radians_from_hardware_for_finger_id(
                    float(self._config.hand_binary_grasp_hardware[i]),
                    self._config.hand_finger_id_order[i],
                )
                for i in range(6)
            ],
            dtype=np.float32,
        )
        d0 = float(np.linalg.norm(hand_rad - reset_rad))
        d1 = float(np.linalg.norm(hand_rad - grasp_rad))
        return 0.0 if d0 <= d1 else 1.0

    def _build_state(self, obs: RobotObservation) -> np.ndarray:
        if len(obs.arm_joint_position) != 7:
            raise ValueError(f"arm_joint_position size mismatch: got {len(obs.arm_joint_position)}, expected 7")
        state = np.zeros((8,), dtype=np.float32)
        state[0:7] = np.asarray(obs.arm_joint_position, dtype=np.float32)
        state[7] = self._build_obs_hand_binary(obs)
        if self._config.state_dim != 8:
            raise ValueError(f"act_exp3_1_state_dim must be 8 for this layout, got {self._config.state_dim}")
        return state

    def _build_batch(self, obs: RobotObservation) -> Dict[str, torch.Tensor]:
        obs_t: Dict[str, torch.Tensor] = {
            self._config.state_key: torch.from_numpy(self._build_state(obs)).unsqueeze(0).to(self._torch_device),
        }
        for i, image_key in enumerate(self._config.image_keys):
            rgb_img, depth_img, _ = resolve_observation_images(
                obs,
                self._config.camera_ids[i],
                self._config.depth_camera_ids[i] if self._config.use_depth else "",
                self._node,
                f"_warned_act_exp31_obs_camera_{i}",
                self,
            )
            rgb = self._rgb_to_numpy(rgb_img)
            obs_t[image_key] = (
                torch.from_numpy(rgb).permute(2, 0, 1).contiguous().float().unsqueeze(0) / 255.0
            ).to(self._torch_device)

            if self._config.use_depth:
                if depth_img.height <= 0 or depth_img.width <= 0:
                    raise ValueError(
                        f"depth image missing/empty for camera {self._config.depth_camera_ids[i]!r}"
                    )
                depth_hwc = self._depth_to_numpy_hwc3(depth_img)
                obs_t[self._config.depth_keys[i]] = (
                    torch.from_numpy(depth_hwc).permute(2, 0, 1).contiguous().float().unsqueeze(0)
                    / 255.0
                ).to(self._torch_device)

        return self._preprocessor(obs_t)

    def _decode_action(self, action_out: Any) -> np.ndarray:
        if isinstance(action_out, dict):
            action_out = action_out.get("action", next(iter(action_out.values())))
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
                f"ACT action shape mismatch: got {action_np.shape}, expected ({self._config.action_dim},)"
            )
        return action_np

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
        return msg

    def infer(self, obs: RobotObservation) -> Optional[WholeBodyAction]:
        batch = self._build_batch(obs)
        with torch.inference_mode():
            action_out = self._policy.select_action(batch)
            action_out = self._postprocessor(action_out)
        action = self._decode_action(action_out)
        return self._to_whole_body_action(action)
