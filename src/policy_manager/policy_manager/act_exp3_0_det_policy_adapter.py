"""ACT exp3_0 adapter + RT-DETRv4 depth pose estimation step."""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
import torch
import torchvision.transforms as T
import yaml
from PIL import Image as PILImage
from rclpy.node import Node

from robot_interfaces.msg import RobotObservation, WholeBodyAction

from policy_manager.act_exp3_0_policy_adapter import ACTExp30PolicyAdapter, ACTExp30PolicyConfig
from policy_manager.observation_camera import resolve_observation_images


class ScaleScoreMode(str, Enum):
    """How to combine width_m and height_m into a single size score."""

    MAX = "max"
    SQRT_AREA = "sqrt_area"


@dataclass(frozen=True)
class DetectedTargetInfo:
    """First successful RT-DETR detection: bbox, depth, base pose, and physical size (meters)."""

    bbox_xyxy: tuple[int, int, int, int]
    center_uv: tuple[int, int]
    depth_m: float
    x: float
    y: float
    z: float
    width_m: float
    height_m: float
    size_score: float


# 三档 6 维硬件寄存器模板，顺序与 `act_exp3_0_det_hand_finger_id_order` 一致（默认 index..thumb_abd）。
# 数值在默认二值 grasp 附近微调，便于现场按包裹再标定。
_SCALE_SMALL_PINCH_HARDWARE: tuple[float, float, float, float, float, float] = (
    700.0,
    700.0,
    500.0,
    500.0,
    500.0,
    0.0,
)
_SCALE_MEDIUM_TRIPOD_HARDWARE: tuple[float, float, float, float, float, float] = (
    600.0,
    600.0,
    500.0,
    500.0,
    500.0,
    0.0,
)
_SCALE_LARGE_WRAP_HARDWARE: tuple[float, float, float, float, float, float] = (
    300.0,
    300.0,
    500.0,
    500.0,
    500.0,
    0.0,
)
# 小包裹档、且抓取未触发（action[7] 低于二值阈值）时给末端加固定偏移（m）；抓取触发时不加偏移。
_SMALL_TIER_EE_OFFSET_M: tuple[float, float, float] = (0.03, 0.0, 0.0)
# scale_test_log 各通道节流间隔（秒），避免与 control_hz 同步刷屏
_SCALE_TEST_LOG_INTERVAL_SEC = 5.0


@dataclass(frozen=True)
class RTDETRPoseConfig:
    enabled: bool
    repo_root: str
    config_path: str
    checkpoint_path: str
    device: str
    camera_id: str
    depth_camera_id: str
    calib_path: str
    calib_direction: str
    use_camera_info_intrinsics: bool
    fx: float
    fy: float
    cx: float
    cy: float
    score_thr: float
    min_area_ratio: float
    max_area_ratio: float
    depth_clip_mm_min: float
    depth_clip_mm_max: float
    environment_state_key: str
    environment_state_dim: int
    env_state_use_detected_xy: bool
    env_state_fallback_to_ee_xy: bool
    env_state_latch_once: bool
    env_state_require_initial_detection: bool
    state_use_detected_xy: bool
    strict_on_failure: bool
    # 尺度自适应闭合（仅本 adapter；与首次检测锁存一致）
    scale_closure_enabled: bool
    scale_small_threshold_m: float
    scale_large_threshold_m: float
    scale_score_mode: ScaleScoreMode
    # 测试阶段：每次成功检测 / 每次尺度闭合下发 grasp 时打印位置、size_score、档位
    scale_test_log: bool


class ACTExp30DetPolicyAdapter(ACTExp30PolicyAdapter):
    """exp3_0 plus one detection-depth target position estimation step."""

    def __init__(
        self, node: Node, config: ACTExp30PolicyConfig, det_config: RTDETRPoseConfig
    ) -> None:
        self._det_config = det_config
        self._detector: Optional[Any] = None
        self._detector_device = "cpu"
        self._warned_detector_infer = False
        self._warned_env_key_missing = False
        self._warned_incomplete_observation = False
        self._warned_wait_initial_detection = False
        self._env_state_cached: Optional[np.ndarray] = None
        self._scale_target_cached: Optional[DetectedTargetInfo] = None
        self._scale_test_log_last_mono: Dict[str, float] = {}
        super().__init__(node=node, config=config)
        self._expects_environment_state = self._det_config.environment_state_key in set(
            getattr(getattr(self._policy, "config", None), "input_features", {}).keys()
        )
        if self._det_config.enabled:
            self._detector, self._detector_device = self._load_detector_bundle(self._det_config)

    @classmethod
    def _load_configs_from_node(
        cls, node: Node, p: str
    ) -> tuple[ACTExp30PolicyConfig, RTDETRPoseConfig]:
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
        camera_ids = list(node.declare_parameter(f"{p}_camera_ids", ["cam1", "cam3", "cam4"]).value or [])
        depth_camera_ids = list(
            node.declare_parameter(f"{p}_depth_camera_ids", ["cam1", "cam3", "cam4"]).value or []
        )
        use_depth = bool(node.declare_parameter(f"{p}_use_depth", True).value)
        state_key = node.declare_parameter(f"{p}_state_key", "observation.state").value
        img_h = int(node.declare_parameter(f"{p}_img_h", 480).value)
        img_w = int(node.declare_parameter(f"{p}_img_w", 640).value)
        depth_clip_mm_min = float(node.declare_parameter(f"{p}_depth_clip_mm_min", 100.0).value)
        depth_clip_mm_max = float(node.declare_parameter(f"{p}_depth_clip_mm_max", 8000.0).value)
        state_dim = int(node.declare_parameter(f"{p}_state_dim", 8).value)
        action_dim = int(node.declare_parameter(f"{p}_action_dim", 8).value)
        arm_reference_frame = node.declare_parameter("arm_reference_frame", "fr3_link0").value
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

        config = ACTExp30PolicyConfig(
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
            arm_reference_frame=str(arm_reference_frame),
            arm_duration_sec=arm_duration_sec,
            hand_duration_sec=hand_duration_sec,
            control_hz=control_hz,
            hand_finger_id_order=tuple(int(x) for x in fid),
            hand_binary_threshold=threshold,
            hand_binary_reset_hardware=tuple(float(x) for x in reset),
            hand_binary_grasp_hardware=tuple(float(x) for x in grasp),
        )

        det = RTDETRPoseConfig(
            enabled=bool(node.declare_parameter(f"{p}_detector_enabled", True).value),
            repo_root=str(node.declare_parameter(f"{p}_detector_repo_root", "/home/jhr/models/rtv4").value),
            config_path=str(
                node.declare_parameter(
                    f"{p}_detector_config_path",
                    "/home/jhr/models/rtv4/configs/rtv4/rtv4_hgnetv2_s_conveyor_simple_sz_2.yml",
                ).value
            ),
            checkpoint_path=str(
                node.declare_parameter(
                    f"{p}_detector_checkpoint_path", "/home/jhr/models/rtv4/rtv4_ckpt/checkpoint0038.pth"
                ).value
            ),
            device=str(node.declare_parameter(f"{p}_detector_device", "auto").value),
            camera_id=str(node.declare_parameter(f"{p}_detector_camera_id", "cam1").value),
            depth_camera_id=str(node.declare_parameter(f"{p}_detector_depth_camera_id", "cam1").value),
            calib_path=str(node.declare_parameter(f"{p}_detector_calib_path", "").value),
            calib_direction=str(node.declare_parameter(f"{p}_detector_calib_direction", "base_from_camera").value),
            use_camera_info_intrinsics=bool(
                node.declare_parameter(f"{p}_detector_use_camera_info_intrinsics", True).value
            ),
            fx=float(node.declare_parameter(f"{p}_detector_fx", 615.0).value),
            fy=float(node.declare_parameter(f"{p}_detector_fy", 615.0).value),
            cx=float(node.declare_parameter(f"{p}_detector_cx", 320.0).value),
            cy=float(node.declare_parameter(f"{p}_detector_cy", 240.0).value),
            score_thr=float(node.declare_parameter(f"{p}_detector_score_thr", 0.35).value),
            min_area_ratio=float(node.declare_parameter(f"{p}_detector_min_area_ratio", 0.002).value),
            max_area_ratio=float(node.declare_parameter(f"{p}_detector_max_area_ratio", 0.90).value),
            depth_clip_mm_min=float(node.declare_parameter(f"{p}_detector_depth_clip_mm_min", 100.0).value),
            depth_clip_mm_max=float(node.declare_parameter(f"{p}_detector_depth_clip_mm_max", 8000.0).value),
            environment_state_key=str(
                node.declare_parameter(f"{p}_environment_state_key", "observation.environment_state").value
            ),
            environment_state_dim=int(node.declare_parameter(f"{p}_environment_state_dim", 2).value),
            env_state_use_detected_xy=bool(node.declare_parameter(f"{p}_env_state_use_detected_xy", True).value),
            env_state_fallback_to_ee_xy=bool(
                node.declare_parameter(f"{p}_env_state_fallback_to_ee_xy", True).value
            ),
            env_state_latch_once=bool(node.declare_parameter(f"{p}_env_state_latch_once", True).value),
            env_state_require_initial_detection=bool(
                node.declare_parameter(f"{p}_env_state_require_initial_detection", True).value
            ),
            state_use_detected_xy=bool(node.declare_parameter(f"{p}_state_use_detected_xy", False).value),
            strict_on_failure=bool(node.declare_parameter(f"{p}_detector_strict_on_failure", False).value),
            scale_closure_enabled=bool(node.declare_parameter(f"{p}_scale_closure_enabled", False).value),
            # 经验标定：小/中/大代表包裹测得的 size_score 分界（米）；见 README 或实验记录
            scale_small_threshold_m=float(node.declare_parameter(f"{p}_scale_small_threshold_m", 0.13).value),
            scale_large_threshold_m=float(node.declare_parameter(f"{p}_scale_large_threshold_m", 0.23).value),
            scale_score_mode=(
                ScaleScoreMode.SQRT_AREA
                if str(node.declare_parameter(f"{p}_scale_score_mode", "max").value).lower()
                in ("sqrt_area", "sqrt", "geometric_mean")
                else ScaleScoreMode.MAX
            ),
            scale_test_log=bool(node.declare_parameter(f"{p}_scale_test_log", True).value),
        )
        if det.environment_state_dim != 2:
            raise RuntimeError(f"{p}_environment_state_dim must be 2, got {det.environment_state_dim}")
        if det.scale_small_threshold_m >= det.scale_large_threshold_m:
            raise RuntimeError(
                f"{p}_scale_small_threshold_m must be < {p}_scale_large_threshold_m "
                f"({det.scale_small_threshold_m} >= {det.scale_large_threshold_m})"
            )
        return config, det

    @classmethod
    def from_node(cls, node: Node) -> "ACTExp30DetPolicyAdapter":
        p = "act_exp3_0_det"
        config, det = cls._load_configs_from_node(node, p)
        adapter = cls(node=node, config=config, det_config=det)
        node.get_logger().info(
            f"ACT exp3_0 detector policy loaded: checkpoint={config.checkpoint_dir} "
            f"act_device={adapter._torch_device} detector_enabled={det.enabled} detector_device={adapter._detector_device}"
        )
        return adapter

    @staticmethod
    def _resolve_device(requested: str) -> str:
        if requested == "auto":
            return "cuda:0" if torch.cuda.is_available() else "cpu"
        if requested.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError("detector device requests CUDA but CUDA is not available")
        return requested

    def _load_detector_bundle(self, det: RTDETRPoseConfig) -> tuple[Any, str]:
        repo = Path(det.repo_root)
        if not repo.is_dir():
            raise RuntimeError(f"RT-DETR repo_root does not exist: {repo}")
        if not (repo / "engine").is_dir():
            raise RuntimeError(f"RT-DETR repo_root missing engine/: {repo}")

        config_path = Path(det.config_path)
        checkpoint_path = Path(det.checkpoint_path)
        if not config_path.exists():
            raise RuntimeError(f"RT-DETR config file not found: {config_path}")
        if not checkpoint_path.exists():
            raise RuntimeError(f"RT-DETR checkpoint file not found: {checkpoint_path}")
        if not det.calib_path:
            raise RuntimeError("act_exp3_0_det_detector_calib_path is empty")
        if not Path(det.calib_path).exists():
            raise RuntimeError(f"RT-DETR calib file not found: {det.calib_path}")

        import sys

        if str(repo) not in sys.path:
            sys.path.insert(0, str(repo))
        from engine.core import YAMLConfig  # type: ignore[import-not-found]

        device = self._resolve_device(det.device)
        cfg = YAMLConfig(str(config_path), resume=str(checkpoint_path))
        if "HGNetv2" in cfg.yaml_cfg:
            cfg.yaml_cfg["HGNetv2"]["pretrained"] = False

        checkpoint = torch.load(str(checkpoint_path), map_location="cpu")
        if "ema" in checkpoint:
            state = checkpoint["ema"]["module"]
        else:
            state = checkpoint["model"]
        cfg.model.load_state_dict(state)

        class _DetectorModel(torch.nn.Module):
            def __init__(self) -> None:
                super().__init__()
                self.model = cfg.model.deploy()
                self.postprocessor = cfg.postprocessor.deploy()

            def forward(self, images: torch.Tensor, orig_target_sizes: torch.Tensor) -> Any:
                outputs = self.model(images)
                return self.postprocessor(outputs, orig_target_sizes)

        model = _DetectorModel().to(device).eval()
        return model, device

    @staticmethod
    def _choose_bbox(
        boxes: np.ndarray,
        scores: np.ndarray,
        labels: np.ndarray,
        w: int,
        h: int,
        score_thr: float,
        min_area_ratio: float,
        max_area_ratio: float,
    ) -> tuple[Optional[list[int]], dict[str, Any]]:
        order = np.argsort(-scores)
        fallback: Optional[tuple[list[int], dict[str, Any]]] = None
        for idx in order.tolist():
            x1, y1, x2, y2 = [float(v) for v in boxes[idx]]
            x1i = int(round(np.clip(x1, 0, w - 1)))
            y1i = int(round(np.clip(y1, 0, h - 1)))
            x2i = int(round(np.clip(x2, 0, w - 1)))
            y2i = int(round(np.clip(y2, 0, h - 1)))
            x1i, x2i = min(x1i, x2i), max(x1i, x2i)
            y1i, y2i = min(y1i, y2i), max(y1i, y2i)
            if x2i <= x1i or y2i <= y1i:
                continue
            ar = float(((x2i - x1i) * (y2i - y1i)) / max(w * h, 1))
            info: dict[str, Any] = {
                "score": float(scores[idx]),
                "label": int(labels[idx]),
                "area_ratio": ar,
                "index": int(idx),
            }
            if ar < min_area_ratio or ar > max_area_ratio:
                continue
            box = [x1i, y1i, x2i, y2i]
            if fallback is None:
                fallback = (box, dict(info))
            if info["score"] >= score_thr:
                info["source"] = "top_score_thr_area_ok"
                return box, info
        if fallback is not None:
            b, info = fallback
            info["source"] = "fallback_area_ok_low_score"
            return b, info
        return None, {"source": "none_after_area_filter"}

    @staticmethod
    def _load_base_from_camera_transform(calib_path: Path, direction: str) -> tuple[np.ndarray, np.ndarray]:
        data = yaml.safe_load(calib_path.read_text(encoding="utf-8"))
        t = data["transform"]["translation"]
        r = data["transform"]["rotation"]
        q = np.asarray([float(r["x"]), float(r["y"]), float(r["z"]), float(r["w"])], dtype=np.float64)
        qn = float(np.linalg.norm(q))
        if qn < 1e-12:
            rot = np.eye(3, dtype=np.float64)
        else:
            x, y, z, w = q / qn
            rot = np.array(
                [
                    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
                ],
                dtype=np.float64,
            )
        trans = np.array([float(t["x"]), float(t["y"]), float(t["z"])], dtype=np.float64)
        if direction == "base_from_camera":
            return rot, trans
        if direction == "camera_from_base":
            ri = rot.T
            return ri, -ri @ trans
        raise RuntimeError(f"Unsupported calib direction: {direction}")

    @staticmethod
    def _depth_u8_to_m(depth_u8: float, clip_min_mm: float, clip_max_mm: float) -> float:
        depth_mm = clip_min_mm + (float(depth_u8) / 255.0) * (clip_max_mm - clip_min_mm)
        return float(depth_mm / 1000.0)

    @staticmethod
    def _resolve_depth_u8_with_fallbacks(
        depth_hwc_u8: np.ndarray,
        box_xyxy: Optional[list[int]],
        u: int,
        v: int,
        *,
        patch_radius: int = 3,
        center_invalid_u8: float = 1.0,
        zero_ratio_trigger: float = 0.30,
        normal_percentile: float = 50.0,
        occluded_percentile: float = 75.0,
        bbox_fallback_percentile: float = 75.0,
        low_u8_trigger: float = 6.0,
        low_u8_min_gap: float = 4.0,
    ) -> float:
        if depth_hwc_u8.ndim == 3:
            d = depth_hwc_u8[:, :, 0].astype(np.float64)
        else:
            d = depth_hwc_u8.astype(np.float64)
        h, w = d.shape
        uu = int(np.clip(u, 0, w - 1))
        vv = int(np.clip(v, 0, h - 1))

        r = max(int(patch_radius), 0)
        x1, x2 = max(0, uu - r), min(w, uu + r + 1)
        y1, y2 = max(0, vv - r), min(h, vv + r + 1)
        patch = d[y1:y2, x1:x2].reshape(-1)
        center_u8 = float(d[vv, uu])
        du8 = center_u8
        if patch.size == 0:
            patch_nonzero_count = 0
        else:
            zero_ratio = float(np.mean(patch <= 0.0))
            nz = patch[patch > 0.0]
            patch_nonzero_count = int(nz.size)
            if nz.size > 0:
                center_bad = center_u8 <= float(center_invalid_u8)
                patch_bad = zero_ratio >= float(zero_ratio_trigger)
                pct = float(occluded_percentile) if (center_bad or patch_bad) else float(normal_percentile)
                du8 = float(np.percentile(nz, pct))

        if isinstance(box_xyxy, list) and len(box_xyxy) == 4:
            bx1, by1, bx2, by2 = [int(vv) for vv in box_xyxy]
            bx1, bx2 = int(np.clip(min(bx1, bx2), 0, w - 1)), int(np.clip(max(bx1, bx2), 0, w - 1))
            by1, by2 = int(np.clip(min(by1, by2), 0, h - 1)), int(np.clip(max(by1, by2), 0, h - 1))
            crop = d[by1 : by2 + 1, bx1 : bx2 + 1].reshape(-1)
            nz_bbox = crop[crop > 0.0]
            if nz_bbox.size > 0:
                bbox_du8 = float(np.percentile(nz_bbox, float(bbox_fallback_percentile)))
                if patch_nonzero_count <= 0:
                    du8 = bbox_du8
                if du8 <= float(low_u8_trigger) and (bbox_du8 - du8) >= float(low_u8_min_gap):
                    du8 = bbox_du8
        return du8

    @staticmethod
    def _pixel_to_base_xyz(
        u: int,
        v: int,
        depth_m: float,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        rot_base_cam: np.ndarray,
        trans_base_cam: np.ndarray,
    ) -> tuple[float, float, float]:
        x_cam = (float(u) - cx) / max(float(fx), 1e-6) * depth_m
        y_cam = (float(v) - cy) / max(float(fy), 1e-6) * depth_m
        z_cam = depth_m
        p_cam = np.array([x_cam, y_cam, z_cam], dtype=np.float64)
        p_base = rot_base_cam @ p_cam + trans_base_cam
        return float(p_base[0]), float(p_base[1]), float(p_base[2])

    def _infer_target_pose_and_scale(self, obs: RobotObservation) -> DetectedTargetInfo:
        assert self._detector is not None
        rgb_img, depth_img, camera_info = resolve_observation_images(
            obs,
            self._det_config.camera_id,
            self._det_config.depth_camera_id,
            self._node,
            "_warned_act_exp30_det_camera",
            self,
        )
        rgb = self._rgb_to_numpy(rgb_img)
        depth_hwc = self._depth_to_numpy_hwc3(depth_img)
        pil = PILImage.fromarray(rgb)
        w, h = pil.size
        tf = T.Compose([T.Resize((640, 640)), T.ToTensor()])
        x = tf(pil).unsqueeze(0).to(self._detector_device)
        orig_size = torch.tensor([[w, h]], device=self._detector_device)
        with torch.no_grad():
            labels, boxes, scores = self._detector(x, orig_size)
        box, _ = self._choose_bbox(
            boxes[0].detach().cpu().numpy(),
            scores[0].detach().cpu().numpy(),
            labels[0].detach().cpu().numpy(),
            w=w,
            h=h,
            score_thr=self._det_config.score_thr,
            min_area_ratio=self._det_config.min_area_ratio,
            max_area_ratio=self._det_config.max_area_ratio,
        )
        if box is None:
            raise RuntimeError("RT-DETR failed to find a valid bbox")
        x1, y1, x2, y2 = box
        u = int(round((x1 + x2) * 0.5))
        v = int(round((y1 + y2) * 0.5))
        du8 = self._resolve_depth_u8_with_fallbacks(
            depth_hwc,
            box,
            u,
            v,
        )
        depth_m = self._depth_u8_to_m(
            du8,
            self._det_config.depth_clip_mm_min,
            self._det_config.depth_clip_mm_max,
        )
        fx, fy, cx, cy = (
            self._det_config.fx,
            self._det_config.fy,
            self._det_config.cx,
            self._det_config.cy,
        )
        if self._det_config.use_camera_info_intrinsics and len(camera_info.k) >= 9:
            fx = float(camera_info.k[0]) if camera_info.k[0] > 1e-6 else fx
            fy = float(camera_info.k[4]) if camera_info.k[4] > 1e-6 else fy
            cx = float(camera_info.k[2]) if camera_info.k[2] > 1e-6 else cx
            cy = float(camera_info.k[5]) if camera_info.k[5] > 1e-6 else cy
        bw_px = float(x2 - x1)
        bh_px = float(y2 - y1)
        width_m = bw_px / max(float(fx), 1e-6) * depth_m
        height_m = bh_px / max(float(fy), 1e-6) * depth_m
        if self._det_config.scale_score_mode == ScaleScoreMode.SQRT_AREA:
            size_score = float(np.sqrt(max(width_m * height_m, 0.0)))
        else:
            size_score = float(max(width_m, height_m))
        rot, trans = self._load_base_from_camera_transform(
            Path(self._det_config.calib_path),
            self._det_config.calib_direction,
        )
        bx, by, bz = self._pixel_to_base_xyz(u, v, depth_m, fx, fy, cx, cy, rot, trans)
        info = DetectedTargetInfo(
            bbox_xyxy=(int(x1), int(y1), int(x2), int(y2)),
            center_uv=(u, v),
            depth_m=float(depth_m),
            x=bx,
            y=by,
            z=bz,
            width_m=float(width_m),
            height_m=float(height_m),
            size_score=size_score,
        )
        self._log_scale_detection_test(info)
        return info

    def _infer_target_base_xyz(self, obs: RobotObservation) -> tuple[float, float, float]:
        info = self._infer_target_pose_and_scale(obs)
        return info.x, info.y, info.z

    def _maybe_latch_scale_target(self, info: DetectedTargetInfo) -> None:
        """与 environment_state 一致：首次成功检测后锁存尺度，本次运行不变。"""
        if self._scale_target_cached is not None:
            return
        self._scale_target_cached = info

    def _scale_hardware_for_size_score(self, size_score: float) -> tuple[float, float, float, float, float, float]:
        ts = float(self._det_config.scale_small_threshold_m)
        tl = float(self._det_config.scale_large_threshold_m)
        if size_score < ts:
            return _SCALE_SMALL_PINCH_HARDWARE
        if size_score < tl:
            return _SCALE_MEDIUM_TRIPOD_HARDWARE
        return _SCALE_LARGE_WRAP_HARDWARE

    def _scale_tier_label_for_score(self, size_score: float) -> str:
        ts = float(self._det_config.scale_small_threshold_m)
        tl = float(self._det_config.scale_large_threshold_m)
        if size_score < ts:
            return "S0_小捏持"
        if size_score < tl:
            return "S1_中三指"
        return "S2_大包覆"

    def _is_small_tier(self, size_score: float) -> bool:
        return size_score < float(self._det_config.scale_small_threshold_m)

    def _emit_scale_test_log_throttled(self, channel: str, message: str) -> None:
        if not self._det_config.scale_test_log:
            return
        now = time.monotonic()
        t0 = self._scale_test_log_last_mono.get(channel)
        if t0 is not None and (now - t0) < _SCALE_TEST_LOG_INTERVAL_SEC:
            return
        self._scale_test_log_last_mono[channel] = now
        self._node.get_logger().info(message)

    def _log_scale_detection_test(self, info: DetectedTargetInfo) -> None:
        if not self._det_config.scale_test_log:
            return
        tier = self._scale_tier_label_for_score(info.size_score)
        self._emit_scale_test_log_throttled(
            "detection",
            f"[act_exp3_0_det 检测] base_xyz_m=({info.x:.4f}, {info.y:.4f}, {info.z:.4f}) | "
            f"size_score={info.size_score:.4f} (width_m={info.width_m:.4f}, height_m={info.height_m:.4f}) | "
            f"档位={tier} | depth_m={info.depth_m:.4f} | uv={info.center_uv} | bbox={info.bbox_xyxy}",
        )

    def _to_whole_body_action(self, action: np.ndarray) -> WholeBodyAction:
        msg = super()._to_whole_body_action(action)
        if not self._det_config.scale_closure_enabled:
            return msg
        cache = self._scale_target_cached
        if cache is None:
            return msg
        grasp_on = float(action[7]) >= float(self._config.hand_binary_threshold)
        if self._is_small_tier(cache.size_score) and (not grasp_on or grasp_on):
            dx, dy, dz = _SMALL_TIER_EE_OFFSET_M
            msg.arm.cartesian_pose.position.x += float(dx)
            msg.arm.cartesian_pose.position.y += float(dy)
            msg.arm.cartesian_pose.position.z += float(dz)
            if self._det_config.scale_test_log:
                self._emit_scale_test_log_throttled(
                    "offset",
                    f"[act_exp3_0_det] 小包裹末端偏移已应用（未抓取）offset_m=({_SMALL_TIER_EE_OFFSET_M[0]:.3f}, "
                    f"{_SMALL_TIER_EE_OFFSET_M[1]:.3f}, {_SMALL_TIER_EE_OFFSET_M[2]:.3f})",
                )
        if not grasp_on:
            return msg
        if self._det_config.scale_test_log:
            tier = self._scale_tier_label_for_score(cache.size_score)
            self._emit_scale_test_log_throttled(
                "grasp",
                f"[act_exp3_0_det grasp] 档位={tier} | size_score={cache.size_score:.4f} | "
                f"base_xyz_m=({cache.x:.4f}, {cache.y:.4f}, {cache.z:.4f}) | action[7]={float(action[7]):.4f}",
            )
        tier_hw = self._scale_hardware_for_size_score(cache.size_score)
        hand = np.asarray(
            [self._normalized_from_hardware(float(k)) for k in tier_hw],
            dtype=np.float32,
        )
        msg.hand.joint_position = [float(x) for x in hand]
        return msg

    def _build_environment_state(self, obs: RobotObservation) -> np.ndarray:
        if self._det_config.env_state_latch_once and self._env_state_cached is not None:
            return self._env_state_cached.copy()
        env = np.zeros((2,), dtype=np.float32)
        try:
            info = self._infer_target_pose_and_scale(obs)
            env[0] = float(info.x)
            env[1] = float(info.y)
            self._maybe_latch_scale_target(info)
            self._warned_detector_infer = False
            if self._det_config.env_state_latch_once:
                self._env_state_cached = env.copy()
            return env
        except Exception as exc:
            if self._det_config.strict_on_failure:
                raise
            if self._det_config.env_state_fallback_to_ee_xy:
                env[0] = float(obs.ee_pose.position.x)
                env[1] = float(obs.ee_pose.position.y)
            if not self._warned_detector_infer:
                self._node.get_logger().warn(
                    f"RT-DETR pose estimation failed, fallback env_state: {exc}"
                )
                self._warned_detector_infer = True
            if self._det_config.env_state_latch_once and not self._det_config.env_state_require_initial_detection:
                self._env_state_cached = env.copy()
            return env

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
                f"_warned_act_exp30_det_obs_camera_{i}",
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

        if self._det_config.enabled and self._det_config.env_state_use_detected_xy:
            if self._expects_environment_state:
                env = self._build_environment_state(obs)
                obs_t[self._det_config.environment_state_key] = (
                    torch.from_numpy(env).unsqueeze(0).to(self._torch_device)
                )
            elif not self._warned_env_key_missing:
                self._node.get_logger().warn(
                    f"Model input_features does not contain {self._det_config.environment_state_key!r}; "
                    "skip injecting environment_state."
                )
                self._warned_env_key_missing = True

        return self._preprocessor(obs_t)

    def _build_state(self, obs: RobotObservation) -> np.ndarray:
        state = super()._build_state(obs)
        if not self._det_config.enabled:
            return state
        if not self._det_config.state_use_detected_xy:
            return state
        try:
            info = self._infer_target_pose_and_scale(obs)
            self._maybe_latch_scale_target(info)
        except Exception as exc:
            if self._det_config.strict_on_failure:
                raise
            if not self._warned_detector_infer:
                self._node.get_logger().warn(f"RT-DETR pose estimation failed, fallback to ee_pose xy: {exc}")
                self._warned_detector_infer = True
            return state
        self._warned_detector_infer = False
        if self._det_config.state_use_detected_xy:
            state[0] = float(info.x)
            state[1] = float(info.y)
        return state

    def _observation_ready(self, obs: RobotObservation) -> tuple[bool, str]:
        if len(obs.hand_joint_position) != 6:
            return False, "hand_joint_position is not ready (expected 6)."
        for i, cid in enumerate(self._config.camera_ids):
            did = self._config.depth_camera_ids[i] if self._config.use_depth else ""
            rgb_img, depth_img, _ = resolve_observation_images(
                obs,
                cid,
                did,
                self._node,
                None,
                None,
            )
            if rgb_img.height <= 0 or rgb_img.width <= 0:
                return False, f"rgb image for camera {cid!r} is empty."
            if self._config.use_depth and (depth_img.height <= 0 or depth_img.width <= 0):
                return False, f"depth image for camera {did or cid!r} is empty."
        if self._det_config.enabled and (
            self._det_config.env_state_use_detected_xy or self._det_config.scale_closure_enabled
        ):
            rgb_img, depth_img, _ = resolve_observation_images(
                obs,
                self._det_config.camera_id,
                self._det_config.depth_camera_id,
                self._node,
                None,
                None,
            )
            if rgb_img.height <= 0 or rgb_img.width <= 0:
                return False, f"detector rgb image for {self._det_config.camera_id!r} is empty."
            if depth_img.height <= 0 or depth_img.width <= 0:
                return False, f"detector depth image for {self._det_config.depth_camera_id!r} is empty."
        return True, ""

    def infer(self, obs: RobotObservation) -> Optional[WholeBodyAction]:
        ready, reason = self._observation_ready(obs)
        if not ready:
            if not self._warned_incomplete_observation:
                self._node.get_logger().warn(
                    f"Observation is incomplete, skip ACT inference this cycle: {reason}"
                )
                self._warned_incomplete_observation = True
            return None
        self._warned_incomplete_observation = False
        need_env_initial = (
            self._det_config.env_state_use_detected_xy
            and self._det_config.env_state_latch_once
            and self._env_state_cached is None
        )
        need_scale_initial = self._det_config.scale_closure_enabled and self._scale_target_cached is None
        if (
            self._det_config.enabled
            and self._det_config.env_state_require_initial_detection
            and (need_env_initial or need_scale_initial)
        ):
            try:
                info = self._infer_target_pose_and_scale(obs)
                self._maybe_latch_scale_target(info)
                if need_env_initial:
                    self._env_state_cached = np.array([info.x, info.y], dtype=np.float32)
                self._warned_wait_initial_detection = False
            except Exception as exc:
                if not self._warned_wait_initial_detection:
                    self._node.get_logger().warn(
                        f"Waiting initial successful detection before ACT inference: {exc}"
                    )
                    self._warned_wait_initial_detection = True
                return None
        return super().infer(obs)
