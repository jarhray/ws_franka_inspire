"""Resolve RGB / depth / camera_info from RobotObservation (multi-camera or legacy)."""

from __future__ import annotations

from typing import Optional, Tuple

from rclpy.node import Node
from robot_interfaces.msg import RobotObservation
from sensor_msgs.msg import CameraInfo, Image


def _find_camera_index(camera_ids: list, camera_id: str) -> int:
    if not camera_id or not camera_ids:
        return -1
    try:
        return list(camera_ids).index(camera_id)
    except ValueError:
        return -1


def resolve_observation_images(
    obs: RobotObservation,
    camera_id: str,
    depth_camera_id: str = "",
    node: Optional[Node] = None,
    log_warn_once_attr: Optional[str] = None,
    self_obj: Optional[object] = None,
) -> Tuple[Image, Image, CameraInfo]:
    """
    Return (rgb, depth, camera_info) for policy adapters.

    - If ``camera_id`` is empty: use legacy ``obs.rgb_image`` / ``obs.depth_image`` / ``obs.camera_info``.
    - Else: look up parallel arrays ``obs.camera_ids`` / ``obs.rgb_images`` / …
    - ``depth_camera_id``: if non-empty, depth (and depth-aligned camera_info) come from that slot;
      RGB still from ``camera_id``. If empty, depth matches ``camera_id`` slot.
    """
    cid = (camera_id or "").strip()
    did = (depth_camera_id or "").strip()

    if not cid:
        return obs.rgb_image, obs.depth_image, obs.camera_info

    ids = list(obs.camera_ids)
    if not ids:
        _maybe_warn(
            self_obj,
            node,
            log_warn_once_attr,
            f"policy requested camera_id={cid!r} but RobotObservation.camera_ids is empty; "
            "using legacy rgb_image/depth_image.",
        )
        return obs.rgb_image, obs.depth_image, obs.camera_info

    ir = _find_camera_index(ids, cid)
    if ir < 0:
        _maybe_warn(
            self_obj,
            node,
            log_warn_once_attr,
            f"camera_id={cid!r} not in observation.camera_ids={ids}; using legacy images.",
        )
        return obs.rgb_image, obs.depth_image, obs.camera_info

    rgb = obs.rgb_images[ir] if ir < len(obs.rgb_images) else Image()
    info_rgb = obs.camera_infos[ir] if ir < len(obs.camera_infos) else CameraInfo()

    id_depth = did if did else cid
    dr = _find_camera_index(ids, id_depth)
    if dr < 0:
        _maybe_warn(
            self_obj,
            node,
            log_warn_once_attr,
            f"depth_camera_id={id_depth!r} not in observation.camera_ids={ids}; using legacy depth.",
        )
        depth = obs.depth_image
    else:
        depth = obs.depth_images[dr] if dr < len(obs.depth_images) else Image()

    # Third slot: RGB camera intrinsics (matches legacy ``obs.camera_info`` semantics).
    return rgb, depth, info_rgb


def _maybe_warn(
    self_obj: Optional[object],
    node: Optional[Node],
    log_warn_once_attr: Optional[str],
    text: str,
) -> None:
    if node is None:
        return
    if self_obj is not None and log_warn_once_attr:
        if getattr(self_obj, log_warn_once_attr, False):
            return
        setattr(self_obj, log_warn_once_attr, True)
    node.get_logger().warn(text)
