#!/usr/bin/env python3
"""
阶段 0 离线诊断：ACT exp1_2（8D 绝对位姿 + 二值夹爪）

建议使用项目虚拟环境运行（与真机 policy 依赖一致），例如：
  /home/jhr/ws_franka_inspire/.venv/bin/python /home/jhr/ws_franka_inspire/scripts/act_exp1_2_offline_diag.py ...

功能概要：
  - 从 LeRobot 数据集读取帧（parquet + 视频），与 checkpoint 内 policy_preprocessor 一致地做推理；
  - 逐帧：预测 vs GT action 的 MSE/L1、四元数范数、z、gripper、|action[:7]-state[:7]|（「复制」距离）；
  - 复制基线：用 state 当作「预测」与 GT action 对比（检测「抄状态」是否足够好）；
  - 图像：对比「训练时链路」prepare_observation + preprocessor 与真机 adapter 式 resize（area）后的统计量。

依赖：与 policy_manager 相同（lerobot、torch、pandas、pyarrow 等），需在含 lerobot 的 venv 中运行。

若解码 mp4 报 Permission denied（常见于属主为其它用户且权限为 600），在机器上执行：
  sudo bash ws_franka_inspire/scripts/fix_exp1_2_video_permissions.sh
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
import torch
import torch.nn.functional as F

# ---------------------------------------------------------------------------
# 几何与标量指标（与 act_exp1_2_policy_adapter 布局一致：xyz + xyzw + binary）
# ---------------------------------------------------------------------------


def quat_norm(v: np.ndarray) -> float:
    q = np.asarray(v, dtype=np.float64).reshape(4)
    return float(np.linalg.norm(q))


def metrics_block(state: np.ndarray, gt_action: np.ndarray, pred: np.ndarray | None) -> dict[str, Any]:
    s = np.asarray(state, dtype=np.float64).ravel()
    g = np.asarray(gt_action, dtype=np.float64).ravel()
    out: dict[str, Any] = {
        "state_z": float(s[2]),
        "gt_z": float(g[2]),
        "state_quat_norm": quat_norm(s[3:7]),
        "gt_quat_norm": quat_norm(g[3:7]),
        "gripper_state": float(s[7]),
        "gripper_gt": float(g[7]),
        "abs_diff_state_gt_first7": float(np.linalg.norm(s[:7] - g[:7])),
        "copy_mse_vs_gt": float(np.mean((s - g) ** 2)),
        "copy_l1_vs_gt": float(np.mean(np.abs(s - g))),
    }
    if pred is not None:
        p = np.asarray(pred, dtype=np.float64).ravel()
        out["pred_z"] = float(p[2])
        out["pred_quat_norm"] = quat_norm(p[3:7])
        out["gripper_pred"] = float(p[7])
        out["mse_pred_vs_gt"] = float(np.mean((p - g) ** 2))
        out["l1_pred_vs_gt"] = float(np.mean(np.abs(p - g)))
        out["abs_diff_pred_gt_first7"] = float(np.linalg.norm(p[:7] - g[:7]))
    return out


def tensor_stats(t: torch.Tensor) -> dict[str, float]:
    x = t.detach().float().cpu()
    return {
        "shape": tuple(x.shape),
        "dtype": str(x.dtype),
        "min": float(x.min().item()),
        "max": float(x.max().item()),
        "mean": float(x.mean().item()),
        "std": float(x.std().item()),
    }


def adapter_style_image_chw(
    img_hwc_uint8: np.ndarray,
    img_h: int,
    img_w: int,
) -> torch.Tensor:
    """对齐 policy_manager 中 ACTExp12PolicyAdapter._build_batch 的图像分支（无 MEAN_STD）。"""
    rgb = torch.from_numpy(img_hwc_uint8).permute(2, 0, 1).unsqueeze(0).float() / 255.0
    rgb = F.interpolate(rgb, size=(img_h, img_w), mode="area")
    return rgb


def load_frames_indices(n_total: int, args: argparse.Namespace) -> list[int]:
    if args.indices is not None:
        idxs = [int(x) for x in args.indices.split(",") if x.strip()]
        for i in idxs:
            if i < 0 or i >= n_total:
                raise SystemExit(f"index {i} out of range [0, {n_total})")
        return idxs
    out: list[int] = []
    i = int(args.start)
    while i < n_total and len(out) < int(args.max_frames):
        out.append(i)
        i += int(args.stride)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description="ACT exp1_2 离线诊断（预测 vs GT、复制基线、图像预处理对比）")
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=Path("/home/jhr/data_recorded/exp1_2"),
        help="含 meta/info.json、data/*.parquet、videos/ 的 LeRobot v3 数据集根目录",
    )
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=None,
        help="ACT pretrained_model 目录（含 config.json、model.safetensors、policy_preprocessor.json）",
    )
    parser.add_argument(
        "--repo-id",
        type=str,
        default="local_exp1_2",
        help="LeRobotDataset 的 repo_id（仅元数据标识，可任意字符串）",
    )
    parser.add_argument("--device", type=str, default="cuda", help="cuda | cpu | auto")
    parser.add_argument(
        "--video-backend",
        type=str,
        default=None,
        help="视频解码后端：pyav / torchcodec / ...；默认由 lerobot 选择",
    )
    parser.add_argument("--image-key", type=str, default="observation.images.rs_color")
    parser.add_argument("--state-key", type=str, default="observation.state")
    parser.add_argument("--action-key", type=str, default="action")
    parser.add_argument("--img-h", type=int, default=480)
    parser.add_argument("--img-w", type=int, default=640)
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--max-frames", type=int, default=20)
    parser.add_argument(
        "--indices",
        type=str,
        default=None,
        help="逗号分隔的全局帧索引，若指定则忽略 start/stride/max-frames",
    )
    parser.add_argument(
        "--parquet-only",
        action="store_true",
        help="不加载视频与策略，仅从 parquet 统计 state/action（复制基线、四元数等）",
    )
    parser.add_argument(
        "--no-policy",
        action="store_true",
        help="加载图像但不加载 ACT 权重（仅数据集与复制基线/几何指标）",
    )
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    if args.parquet_only:
        rows = _parquet_only_metrics(args)
        print(json.dumps(rows, indent=2, ensure_ascii=False))
        return

    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors
    from lerobot.policies.utils import prepare_observation_for_inference
    from lerobot.utils.control_utils import predict_action

    root = args.dataset_root.expanduser().resolve()
    if not (root / "meta" / "info.json").is_file():
        print(f"ERROR: missing {root / 'meta' / 'info.json'}", file=sys.stderr)
        raise SystemExit(2)

    ds_kw: dict[str, Any] = {"repo_id": args.repo_id, "root": root}
    if args.video_backend:
        ds_kw["video_backend"] = args.video_backend

    print(f"Loading LeRobotDataset root={root} ...", file=sys.stderr)
    try:
        dataset = LeRobotDataset(**ds_kw)
    except Exception as exc:
        print(f"ERROR: failed to open dataset: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc

    n_total = len(dataset)
    indices = load_frames_indices(n_total, args)
    if not indices:
        print("No frames selected.", file=sys.stderr)
        raise SystemExit(0)

    policy = None
    preprocessor = None
    postprocessor = None
    device_torch: torch.device | None = None
    policy_cfg: Any = None

    if not args.no_policy:
        if args.checkpoint is None or not args.checkpoint.is_dir():
            print("ERROR: --checkpoint 必须是包含训练产物的目录", file=sys.stderr)
            raise SystemExit(2)
        ckpt = str(args.checkpoint.expanduser().resolve())
        print(f"Loading ACT policy from {ckpt} ...", file=sys.stderr)
        policy_cfg = PreTrainedConfig.from_pretrained(ckpt)
        dev = args.device
        if dev == "auto":
            dev = "cuda" if torch.cuda.is_available() else "cpu"
        policy_cfg.device = dev
        policy = ACTPolicy.from_pretrained(ckpt, config=policy_cfg)
        policy.eval()
        device_torch = torch.device(dev)
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=policy_cfg,
            pretrained_path=ckpt,
            preprocessor_overrides={
                "device_processor": {"device": str(device_torch)},
            },
        )
    elif args.checkpoint is not None and args.checkpoint.is_dir():
        # 仅加载 preprocessor（与训练一致），不加载权重，用于对比图像归一化而无需 GPU
        ckpt = str(args.checkpoint.expanduser().resolve())
        print(f"Loading preprocessor only from {ckpt} (--no-policy) ...", file=sys.stderr)
        policy_cfg = PreTrainedConfig.from_pretrained(ckpt)
        dev = args.device
        if dev == "auto":
            dev = "cuda" if torch.cuda.is_available() else "cpu"
        policy_cfg.device = dev
        device_torch = torch.device(dev)
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=policy_cfg,
            pretrained_path=ckpt,
            preprocessor_overrides={
                "device_processor": {"device": str(device_torch)},
            },
        )

    results: list[dict[str, Any]] = []

    for idx in indices:
        try:
            item = dataset[idx]
        except Exception as exc:
            print(f"WARN: dataset[{idx}] failed: {exc}", file=sys.stderr)
            continue

        st = item[args.state_key]
        ac = item[args.action_key]
        if isinstance(st, torch.Tensor):
            state_np = st.detach().cpu().numpy().astype(np.float32)
        else:
            state_np = np.asarray(st, dtype=np.float32)
        if isinstance(ac, torch.Tensor):
            gt_action_np = ac.detach().cpu().numpy().astype(np.float32)
        else:
            gt_action_np = np.asarray(ac, dtype=np.float32)
        state_np = state_np.reshape(-1)
        gt_action_np = gt_action_np.reshape(-1)

        img_key = args.image_key
        if img_key not in item:
            print(f"WARN: missing {img_key} at index {idx}", file=sys.stderr)
            continue

        img_t = item[img_key]
        if not isinstance(img_t, torch.Tensor):
            img_t = torch.as_tensor(img_t)
        img_chw = img_t.detach().cpu()
        if img_chw.dim() != 3:
            raise RuntimeError(f"expected CHW image, got shape {tuple(img_chw.shape)}")
        img_hwc = (img_chw.permute(1, 2, 0).numpy() * 255.0).clip(0, 255).astype(np.uint8)

        pred_np: np.ndarray | None = None
        if policy is not None and device_torch is not None and preprocessor is not None and postprocessor is not None:
            obs: dict[str, np.ndarray] = {
                args.state_key: state_np,
                img_key: img_hwc,
            }
            policy.reset()
            use_amp = bool(getattr(policy.config, "use_amp", False))
            with torch.inference_mode():
                action_t = predict_action(
                    observation=obs,
                    policy=policy,
                    device=device_torch,
                    preprocessor=preprocessor,
                    postprocessor=postprocessor,
                    use_amp=use_amp,
                    task=None,
                    robot_type=None,
                )
            if isinstance(action_t, torch.Tensor):
                pred_np = action_t.detach().float().cpu().numpy().reshape(-1)
            else:
                pred_np = np.asarray(action_t, dtype=np.float32).reshape(-1)

        block = metrics_block(state_np, gt_action_np, pred_np)
        block["index"] = idx

        # 图像：训练 preprocessor 后 vs adapter 式
        dev2 = device_torch or torch.device("cpu")
        obs_prep = prepare_observation_for_inference(
            {args.state_key: state_np, img_key: img_hwc},
            dev2,
            task=None,
            robot_type=None,
        )
        if preprocessor is not None:
            proc_in = preprocessor(obs_prep)
            img_after_prep = proc_in[img_key]
            block["image_after_policy_preprocessor"] = tensor_stats(img_after_prep)
        else:
            block["image_after_policy_preprocessor"] = {"skipped": "no checkpoint / preprocessor"}
        block["image_adapter_style_area"] = tensor_stats(
            adapter_style_image_chw(img_hwc, args.img_h, args.img_w)
        )

        results.append(block)

    summary = _summarize(results)
    out = {"per_frame": results, "summary": summary}
    if not results and not args.parquet_only:
        print(
            "ERROR: 没有成功处理任何帧。常见原因：视频文件无读权限、索引越界。"
            "可先用 --parquet-only 检查 state/action，或修复 videos/ 权限后重试。",
            file=sys.stderr,
        )
        raise SystemExit(1)
    print(json.dumps(out, indent=2, ensure_ascii=False))


def _summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    if not rows:
        return {}
    keys = [
        "copy_mse_vs_gt",
        "mse_pred_vs_gt",
        "copy_l1_vs_gt",
        "l1_pred_vs_gt",
        "state_quat_norm",
        "gt_quat_norm",
        "pred_quat_norm",
        "abs_diff_state_gt_first7",
    ]
    summ: dict[str, Any] = {}
    for k in keys:
        vals = [r[k] for r in rows if k in r and r[k] is not None]
        if not vals:
            continue
        a = np.array(vals, dtype=np.float64)
        summ[k] = {
            "mean": float(a.mean()),
            "std": float(a.std()),
            "min": float(a.min()),
            "max": float(a.max()),
        }
    return summ


def _parquet_only_metrics(args: argparse.Namespace) -> dict[str, Any]:
    """不依赖视频：扫描 data/chunk-*/file-*.parquet，统计复制基线与四元数等。"""
    root = args.dataset_root.expanduser().resolve()
    files = sorted(root.glob("data/chunk-*/file-*.parquet"))
    if not files:
        print(f"ERROR: no parquet under {root / 'data'}", file=sys.stderr)
        raise SystemExit(2)

    frames: list[dict[str, Any]] = []
    global_index = 0
    for pf in files:
        df = pd.read_parquet(pf)
        n = len(df)
        for i in range(n):
            st = np.asarray(df[args.state_key].iloc[i], dtype=np.float32).ravel()
            g = np.asarray(df[args.action_key].iloc[i], dtype=np.float32).ravel()
            frames.append(metrics_block(st, g, None))
            frames[-1]["parquet_file"] = pf.name
            frames[-1]["row_in_file"] = i
            frames[-1]["index"] = global_index
            global_index += 1

    n_total = len(frames)
    indices = load_frames_indices(n_total, args)
    selected = [frames[i] for i in indices if i < n_total]
    return {
        "mode": "parquet_only",
        "parquet_files": [str(p) for p in files],
        "per_frame": selected,
        "summary": _summarize(selected),
    }


if __name__ == "__main__":
    main()
