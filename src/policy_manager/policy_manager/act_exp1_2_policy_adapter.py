"""ACT adapter for dataset layout in data_recorded/exp1_2/meta/info.json (8-dim state/action)."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
import torch
from rclpy.node import Node

from robot_interfaces.msg import RobotObservation, WholeBodyAction

from policy_manager.base_policy import BasePolicy
from policy_manager.observation_camera import resolve_observation_images


@dataclass(frozen=True)
class ACTExp12PolicyConfig:
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
    hand_finger_id_order: tuple[int, int, int, int, int, int]
    hand_binary_threshold: float
    hand_binary_reset_hardware: tuple[float, float, float, float, float, float]
    hand_binary_grasp_hardware: tuple[float, float, float, float, float, float]
    obs_camera_id: str = ""


class ACTExp12PolicyAdapter(BasePolicy):
    """Matches exp1_2 / info.json: observation.state(8), action(8)."""

    def __init__(self, node: Node, config: ACTExp12PolicyConfig) -> None:
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
        p = "act_exp1_2"
        checkpoint_dir = node.declare_parameter(f"{p}_checkpoint_dir", "").value
        device = node.declare_parameter(f"{p}_device", "auto").value
        image_key = node.declare_parameter(
            f"{p}_image_key", "observation.images.rs_color"
        ).value
        state_key = node.declare_parameter(f"{p}_state_key", "observation.state").value
        img_h = int(node.declare_parameter(f"{p}_img_h", 480).value)
        img_w = int(node.declare_parameter(f"{p}_img_w", 640).value)
        state_dim = int(node.declare_parameter(f"{p}_state_dim", 8).value)
        action_dim = int(node.declare_parameter(f"{p}_action_dim", 8).value)
        control_hz = float(node.declare_parameter(f"{p}_control_hz", 10.0).value)
        arm_reference_frame = node.declare_parameter("arm_reference_frame", "fr3_link0").value
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
        obs_camera_id = str(node.declare_parameter(f"{p}_camera_id", "").value or "").strip()

        config = ACTExp12PolicyConfig(
            checkpoint_dir=str(checkpoint_dir),
            device=str(device),
            image_key=str(image_key),
            state_key=str(state_key),
            img_h=img_h,
            img_w=img_w,
            state_dim=state_dim,
            action_dim=action_dim,
            arm_reference_frame=str(arm_reference_frame),
            arm_duration_sec=arm_duration_sec,
            hand_duration_sec=hand_duration_sec,
            control_hz=control_hz,
            hand_finger_id_order=tuple(int(x) for x in fid),
            hand_binary_threshold=threshold,
            hand_binary_reset_hardware=tuple(float(x) for x in reset),
            hand_binary_grasp_hardware=tuple(float(x) for x in grasp),
            obs_camera_id=obs_camera_id,
        )
        adapter = cls(node=node, config=config)
        node.get_logger().info(
            f"ACT exp1_2 policy loaded: checkpoint={checkpoint_dir} device={adapter._torch_device} "
            f"image_key={image_key} state_key={state_key} camera_id={config.obs_camera_id!r}"
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
        # inspire_executor: k = (x + 1) * 500  =>  x = k / 500 - 1
        return float(np.clip((k / 500.0) - 1.0, -1.0, 1.0))

    def _resolve_torch_device(self, requested: str) -> str:
        if requested == "auto":
            return "cuda" if torch.cuda.is_available() else "cpu"
        if requested not in ("cpu", "cuda"):
            raise RuntimeError(f"Invalid act_exp1_2_device={requested}, expected auto|cpu|cuda")
        if requested == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("act_exp1_2_device=cuda but CUDA is not available")
        return requested

    def _load_policy_bundle(self, checkpoint_dir: str) -> tuple[Any, Any, Any]:
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies.act.modeling_act import ACTPolicy
        from lerobot.policies.factory import make_pre_post_processors

        if not checkpoint_dir:
            raise RuntimeError("act_exp1_2_checkpoint_dir is empty while policy_type=act_exp1_2")
        ckpt = Path(checkpoint_dir)
        if not ckpt.exists():
            raise RuntimeError(f"ACT exp1_2 checkpoint dir does not exist: {ckpt}")
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

    def _rgb_to_numpy(self, obs: RobotObservation) -> np.ndarray:
        image, _, _ = resolve_observation_images(
            obs,
            self._config.obs_camera_id,
            "",
            self._node,
            "_warned_act_exp12_obs_camera",
            self,
        )
        if image.height <= 0 or image.width <= 0:
            raise ValueError("rgb_image has invalid width/height")
        expected_min_step = int(image.width) * 3
        if image.step < expected_min_step:
            raise ValueError(f"rgb_image step too small: {image.step}, expected >= {expected_min_step}")
        raw = np.frombuffer(image.data, dtype=np.uint8)
        if raw.size != int(image.height) * int(image.step):
            raise ValueError("rgb_image bytes mismatch")
        img = raw.reshape((int(image.height), int(image.step)))[:, : expected_min_step]
        img = img.reshape((int(image.height), int(image.width), 3))
        if image.encoding.lower() == "bgr8":
            img = img[..., ::-1]
        return np.ascontiguousarray(img)

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
        state = np.zeros((8,), dtype=np.float32)
        state[0] = float(obs.ee_pose.position.x)
        state[1] = float(obs.ee_pose.position.y)
        state[2] = float(obs.ee_pose.position.z)
        state[3] = float(obs.ee_pose.orientation.x)
        state[4] = float(obs.ee_pose.orientation.y)
        state[5] = float(obs.ee_pose.orientation.z)
        state[6] = float(obs.ee_pose.orientation.w)
        state[7] = self._build_obs_hand_binary(obs)
        if self._config.state_dim != 8:
            raise ValueError(f"act_exp1_2_state_dim must be 8 for this layout, got {self._config.state_dim}")
        return state

    def _build_batch(self, obs: RobotObservation) -> Dict[str, torch.Tensor]:
        from lerobot.policies.utils import prepare_observation_for_inference

        rgb = self._rgb_to_numpy(obs)
        state = self._build_state(obs)
        obs_np = {
            self._config.image_key: rgb,
            self._config.state_key: state,
        }
        obs_t = prepare_observation_for_inference(
            obs_np,
            torch.device(self._torch_device),
            task=None,
            robot_type=None,
        )
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

