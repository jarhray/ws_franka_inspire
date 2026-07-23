"""ACT adapter for the dexterous egg-grasping policy."""

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
class ACTDex0PolicyConfig:
    checkpoint_dir: str
    device: str
    image_keys: tuple[str, ...]
    camera_ids: tuple[str, ...]
    letterbox_camera_ids: frozenset[str]
    state_key: str
    tactile_key: str
    img_h: int
    img_w: int
    state_dim: int
    tactile_rows: int
    tactile_cols: int
    action_dim: int
    arm_reference_frame: str
    arm_duration_sec: float
    hand_duration_sec: float
    control_hz: float
    hand_finger_id_order: tuple[int, int, int, int, int, int]


class ACTDex0PolicyAdapter(BasePolicy):
    """Deploy the egg-grasping ACT model with RGB and tactile input."""

    def __init__(self, node: Node, config: ACTDex0PolicyConfig) -> None:
        self._node = node
        self._config = config
        self._torch_device = self._resolve_torch_device(config.device)
        policy_bundle = self._load_policy_bundle(config.checkpoint_dir)
        self._policy, self._preprocessor, self._postprocessor = policy_bundle

    @property
    def control_hz(self) -> float:
        return float(self._config.control_hz)

    @classmethod
    def from_node(cls, node: Node) -> "BasePolicy":
        prefix = "act_dex_0"
        checkpoint_dir = node.declare_parameter(
            f"{prefix}_checkpoint_dir", ""
        ).value
        device = node.declare_parameter(f"{prefix}_device", "auto").value
        image_keys = list(
            node.declare_parameter(
                f"{prefix}_image_keys",
                [
                    "observation.images.cam1",
                    "observation.images.cam3",
                    "observation.images.cam4",
                ],
            ).value
            or []
        )
        camera_ids = list(
            node.declare_parameter(
                f"{prefix}_camera_ids", ["cam1", "cam3", "cam4"]
            ).value
            or []
        )
        letterbox_camera_ids = list(
            node.declare_parameter(
                f"{prefix}_letterbox_camera_ids", ["cam4"]
            ).value
            or []
        )
        state_key = node.declare_parameter(
            f"{prefix}_state_key", "observation.state"
        ).value
        tactile_key = node.declare_parameter(
            f"{prefix}_tactile_key", "observation.tactile"
        ).value
        img_h = int(node.declare_parameter(f"{prefix}_img_h", 240).value)
        img_w = int(node.declare_parameter(f"{prefix}_img_w", 320).value)
        state_dim = int(
            node.declare_parameter(f"{prefix}_state_dim", 13).value
        )
        tactile_rows = int(
            node.declare_parameter(f"{prefix}_tactile_rows", 2).value
        )
        tactile_cols = int(
            node.declare_parameter(f"{prefix}_tactile_cols", 5).value
        )
        action_dim = int(
            node.declare_parameter(f"{prefix}_action_dim", 13).value
        )
        arm_reference_frame = node.declare_parameter(
            "arm_reference_frame", "fr3_link0"
        ).value
        arm_duration_sec = float(
            node.declare_parameter("arm_duration_sec", 0.1).value
        )
        hand_duration_sec = float(
            node.declare_parameter("hand_duration_sec", 0.1).value
        )
        control_hz = float(
            node.declare_parameter(f"{prefix}_control_hz", 30.0).value
        )
        finger_ids = list(
            node.declare_parameter(
                f"{prefix}_hand_finger_id_order", [0, 1, 2, 3, 4, 5]
            ).value
        )

        if len(image_keys) == 0:
            raise RuntimeError(f"{prefix}_image_keys must be non-empty")
        if len(camera_ids) != len(image_keys):
            raise RuntimeError(
                f"{prefix}_camera_ids length ({len(camera_ids)}) must match "
                f"{prefix}_image_keys length ({len(image_keys)})"
            )
        unique_camera_ids = {
            str(camera_id).strip() for camera_id in camera_ids
        }
        if len(unique_camera_ids) != len(camera_ids):
            raise RuntimeError(f"{prefix}_camera_ids must be unique")
        if len(finger_ids) != 6:
            raise RuntimeError(
                f"{prefix}_hand_finger_id_order must have length 6, "
                f"got {len(finger_ids)}"
            )
        if sorted(int(value) for value in finger_ids) != list(range(6)):
            raise RuntimeError(
                f"{prefix}_hand_finger_id_order must contain each id 0..5"
            )
        unknown_letterbox_ids = {
            str(value).strip() for value in letterbox_camera_ids
        } - unique_camera_ids
        if unknown_letterbox_ids:
            raise RuntimeError(
                f"{prefix}_letterbox_camera_ids contains unknown cameras: "
                f"{sorted(unknown_letterbox_ids)}"
            )
        if img_h <= 0 or img_w <= 0:
            raise RuntimeError(
                f"{prefix}_img_h and {prefix}_img_w must be positive"
            )
        if state_dim != 13 or action_dim != 13:
            raise RuntimeError(
                f"{prefix} requires state_dim=13 and action_dim=13"
            )
        if tactile_rows != 2 or tactile_cols != 5:
            raise RuntimeError(f"{prefix} requires tactile shape 2x5")

        config = ACTDex0PolicyConfig(
            checkpoint_dir=str(checkpoint_dir),
            device=str(device),
            image_keys=tuple(str(value) for value in image_keys),
            camera_ids=tuple(str(value).strip() for value in camera_ids),
            letterbox_camera_ids=frozenset(
                str(value).strip() for value in letterbox_camera_ids
            ),
            state_key=str(state_key),
            tactile_key=str(tactile_key),
            img_h=img_h,
            img_w=img_w,
            state_dim=state_dim,
            tactile_rows=tactile_rows,
            tactile_cols=tactile_cols,
            action_dim=action_dim,
            arm_reference_frame=str(arm_reference_frame),
            arm_duration_sec=arm_duration_sec,
            hand_duration_sec=hand_duration_sec,
            control_hz=control_hz,
            hand_finger_id_order=tuple(int(value) for value in finger_ids),
        )
        adapter = cls(node=node, config=config)
        node.get_logger().info(
            f"ACT dex 0 policy loaded: checkpoint={checkpoint_dir} "
            f"device={adapter._torch_device} "
            f"camera_ids={list(config.camera_ids)} "
            f"letterbox_camera_ids={sorted(config.letterbox_camera_ids)}"
        )
        return adapter

    def _resolve_torch_device(self, requested: str) -> str:
        if requested == "auto":
            return "cuda" if torch.cuda.is_available() else "cpu"
        if requested not in ("cpu", "cuda"):
            raise RuntimeError(
                f"Invalid act_dex_0_device={requested}, expected auto|cpu|cuda"
            )
        if requested == "cuda" and not torch.cuda.is_available():
            raise RuntimeError(
                "act_dex_0_device=cuda but CUDA is not available"
            )
        return requested

    def _load_policy_bundle(self, checkpoint_dir: str) -> tuple[Any, Any, Any]:
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies.act.modeling_act import ACTPolicy
        from lerobot.policies.factory import make_pre_post_processors

        if not checkpoint_dir:
            raise RuntimeError(
                "act_dex_0_checkpoint_dir is empty while policy_type=act_dex_0"
            )
        checkpoint = Path(checkpoint_dir)
        if not checkpoint.exists():
            raise RuntimeError(
                f"ACT dex 0 checkpoint dir does not exist: {checkpoint}"
            )

        policy_config = PreTrainedConfig.from_pretrained(str(checkpoint))
        policy_config.device = self._torch_device
        policy = ACTPolicy.from_pretrained(
            str(checkpoint), config=policy_config
        )
        policy.to(self._torch_device)
        policy.eval()
        if hasattr(policy, "reset"):
            policy.reset()
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=policy_config,
            pretrained_path=str(checkpoint),
            preprocessor_overrides={
                "device_processor": {"device": self._torch_device},
            },
        )
        return policy, preprocessor, postprocessor

    @staticmethod
    def _normalized_from_hardware(hardware: float) -> float:
        return float(np.clip((hardware / 500.0) - 1.0, -1.0, 1.0))

    @staticmethod
    def _decode_rgb_image(image: Image) -> np.ndarray:
        height = int(image.height)
        width = int(image.width)
        step = int(image.step)
        if height <= 0 or width <= 0:
            raise ValueError("rgb_image has invalid width/height")

        encoding = (image.encoding or "").lower()
        channels = {
            "rgb8": 3,
            "bgr8": 3,
            "8uc3": 3,
            "rgba8": 4,
            "bgra8": 4,
            "mono8": 1,
        }.get(encoding)
        if channels is None:
            raise ValueError(
                f"Unsupported rgb_image encoding: {image.encoding}"
            )
        if step < width * channels:
            raise ValueError(
                f"rgb_image step too small: {step}, "
                f"expected >= {width * channels}"
            )

        raw = np.frombuffer(image.data, dtype=np.uint8)
        if raw.size != height * step:
            raise ValueError(
                f"rgb_image bytes mismatch: got {raw.size}, "
                f"expected {height * step}"
            )
        decoded = raw.reshape(height, step)[:, : width * channels].reshape(
            height, width, channels
        )
        if encoding in ("bgr8", "8uc3"):
            decoded = cv2.cvtColor(decoded, cv2.COLOR_BGR2RGB)
        elif encoding == "rgba8":
            decoded = cv2.cvtColor(decoded, cv2.COLOR_RGBA2RGB)
        elif encoding == "bgra8":
            decoded = cv2.cvtColor(decoded, cv2.COLOR_BGRA2RGB)
        elif encoding == "mono8":
            decoded = cv2.cvtColor(decoded, cv2.COLOR_GRAY2RGB)
        return np.ascontiguousarray(decoded)

    def _resize_rgb(self, image: np.ndarray, camera_id: str) -> np.ndarray:
        target_height = self._config.img_h
        target_width = self._config.img_w
        if camera_id not in self._config.letterbox_camera_ids:
            resized = cv2.resize(
                image,
                (target_width, target_height),
                interpolation=cv2.INTER_AREA,
            )
            return np.ascontiguousarray(resized)

        source_height, source_width = image.shape[:2]
        scale = min(target_width / source_width, target_height / source_height)
        resized_width = max(
            1, min(target_width, round(source_width * scale))
        )
        resized_height = max(
            1, min(target_height, round(source_height * scale))
        )
        interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
        resized = cv2.resize(
            image,
            (resized_width, resized_height),
            interpolation=interpolation,
        )
        output = np.zeros((target_height, target_width, 3), dtype=np.uint8)
        offset_y = (target_height - resized_height) // 2
        offset_x = (target_width - resized_width) // 2
        output[
            offset_y:offset_y + resized_height,
            offset_x:offset_x + resized_width,
        ] = resized
        return np.ascontiguousarray(output)

    def _build_state(self, observation: RobotObservation) -> np.ndarray:
        hardware_position = observation.hand_joint_hardware_position
        if len(hardware_position) != 6:
            raise ValueError(
                "hand_joint_hardware_position size mismatch: "
                f"got {len(hardware_position)}, expected 6"
            )
        hardware = np.asarray(hardware_position, dtype=np.float32)
        if not np.all(np.isfinite(hardware)):
            raise ValueError(
                "hand_joint_hardware_position contains non-finite values"
            )
        hardware_by_finger_id = np.zeros((6,), dtype=np.float32)
        for index, finger_id in enumerate(
            self._config.hand_finger_id_order
        ):
            hardware_by_finger_id[finger_id] = hardware[index]

        state = np.zeros((13,), dtype=np.float32)
        state[:7] = np.asarray(
            [
                observation.ee_pose.position.x,
                observation.ee_pose.position.y,
                observation.ee_pose.position.z,
                observation.ee_pose.orientation.x,
                observation.ee_pose.orientation.y,
                observation.ee_pose.orientation.z,
                observation.ee_pose.orientation.w,
            ],
            dtype=np.float32,
        )
        state[7:13] = hardware_by_finger_id
        return state

    def _build_tactile(self, observation: RobotObservation) -> np.ndarray:
        touch = observation.hand_touch
        if not touch.finger_ids:
            raise ValueError("hand_touch is missing finger_ids")
        if not touch.normal_forces or not touch.tangential_forces:
            raise ValueError(
                "hand_touch is missing normal or tangential forces"
            )

        tactile = np.zeros((2, 5), dtype=np.float32)
        for finger_id, normal_force in zip(
            touch.finger_ids, touch.normal_forces, strict=False
        ):
            index = int(finger_id)
            if 0 <= index < 5:
                tactile[0, index] = float(normal_force)
        for finger_id, tangential_force in zip(
            touch.finger_ids, touch.tangential_forces, strict=False
        ):
            index = int(finger_id)
            if 0 <= index < 5:
                tactile[1, index] = float(tangential_force)
        if not np.all(np.isfinite(tactile)):
            raise ValueError("hand_touch contains non-finite values")
        return tactile

    def _build_batch(
        self, observation: RobotObservation
    ) -> Dict[str, torch.Tensor]:
        state = self._build_state(observation)
        batch: Dict[str, torch.Tensor] = {
            self._config.state_key: torch.from_numpy(state)
            .unsqueeze(0)
            .to(self._torch_device),
            self._config.tactile_key: torch.from_numpy(
                self._build_tactile(observation)
            )
            .unsqueeze(0)
            .to(self._torch_device),
        }
        for index, image_key in enumerate(self._config.image_keys):
            camera_id = self._config.camera_ids[index]
            rgb_image, _, _ = resolve_observation_images(
                observation,
                camera_id,
                "",
                self._node,
                f"_warned_act_dex_0_obs_camera_{index}",
                self,
            )
            decoded_rgb = self._decode_rgb_image(rgb_image)
            rgb = self._resize_rgb(decoded_rgb, camera_id)
            batch[image_key] = (
                torch.from_numpy(rgb)
                .permute(2, 0, 1)
                .contiguous()
                .float()
                .unsqueeze(0)
                .div(255.0)
                .to(self._torch_device)
            )
        return self._preprocessor(batch)

    def _decode_action(self, action_output: Any) -> np.ndarray:
        if isinstance(action_output, dict):
            if "action" in action_output:
                action_output = action_output["action"]
            elif action_output:
                action_output = next(iter(action_output.values()))
            else:
                raise ValueError("ACT output dict is empty")
        if not isinstance(action_output, torch.Tensor):
            raise TypeError(f"ACT output is not Tensor: {type(action_output)}")
        action = action_output.detach().float().cpu()
        if action.ndim == 2:
            action = action[0]
        elif action.ndim == 3:
            action = action[0, 0]
        else:
            raise ValueError(
                f"Unsupported ACT output shape: {tuple(action.shape)}"
            )
        action_array = action.numpy()
        if action_array.shape != (self._config.action_dim,):
            raise ValueError(
                f"ACT action shape mismatch: got {action_array.shape}, "
                f"expected ({self._config.action_dim},)"
            )
        if not np.all(np.isfinite(action_array)):
            raise ValueError("ACT action contains non-finite values")
        return action_array

    def _to_whole_body_action(self, action: np.ndarray) -> WholeBodyAction:
        if action.shape != (13,):
            raise ValueError(
                f"Expected action shape (13,), got {action.shape}"
            )

        message = WholeBodyAction()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.arm.control_mode = 2
        message.arm.is_relative = False
        message.arm.reference_frame = self._config.arm_reference_frame
        message.arm.cartesian_pose.position.x = float(action[0])
        message.arm.cartesian_pose.position.y = float(action[1])
        message.arm.cartesian_pose.position.z = float(action[2])
        message.arm.cartesian_pose.orientation.x = float(action[3])
        message.arm.cartesian_pose.orientation.y = float(action[4])
        message.arm.cartesian_pose.orientation.z = float(action[5])
        message.arm.cartesian_pose.orientation.w = float(action[6])
        message.arm.duration_sec = float(self._config.arm_duration_sec)

        message.hand.control_mode = 0
        message.hand.is_relative = False
        message.hand.joint_position = [
            self._normalized_from_hardware(float(value))
            for value in action[7:13]
        ]
        message.hand.joint_velocity = []
        message.hand.duration_sec = float(self._config.hand_duration_sec)
        return message

    def infer(
        self, observation: RobotObservation
    ) -> Optional[WholeBodyAction]:
        batch = self._build_batch(observation)
        with torch.inference_mode():
            action_output = self._policy.select_action(batch)
            action_output = self._postprocessor(action_output)
        return self._to_whole_body_action(self._decode_action(action_output))
