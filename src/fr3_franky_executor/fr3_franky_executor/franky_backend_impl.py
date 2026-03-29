"""Mock backend and shared helpers (no franky import — mock works without franky)."""

from __future__ import annotations

import math
import queue
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

from geometry_msgs.msg import Pose

ARM_JOINT_POSITION = 0
ARM_JOINT_VELOCITY = 1
ARM_CARTESIAN_POSE = 2
ARM_CARTESIAN_VELOCITY = 3


def _finite(x: float) -> bool:
    return math.isfinite(x)


def _fmt_vec(v: List[float], max_items: int = 3) -> str:
    if not v:
        return '[]'
    clipped = v[:max_items]
    suffix = '...' if len(v) > max_items else ''
    return '[' + ', '.join(f'{x:.4f}' for x in clipped) + suffix + ']'


def _duration_from_sec(sec: float):
    """franky.Duration uses integer milliseconds; imported lazily by real backend."""
    from franky import Duration

    if not _finite(sec) or sec <= 0.0:
        sec = 0.001
    ms = max(1, int(round(sec * 1000.0)))
    return Duration(ms)


def _joint_array7(values: List[float]) -> np.ndarray:
    out = np.zeros((7, 1), dtype=np.float64)
    for i in range(min(7, len(values))):
        v = float(values[i])
        out[i, 0] = v if _finite(v) else 0.0
    return out


def _pose_to_affine(pose: Pose):
    from franky import Affine

    t = np.array([[pose.position.x], [pose.position.y], [pose.position.z]], dtype=np.float64)
    q = np.array(
        [
            [pose.orientation.x],
            [pose.orientation.y],
            [pose.orientation.z],
            [pose.orientation.w],
        ],
        dtype=np.float64,
    )
    return Affine(translation=t, quaternion=q)


@dataclass
class BackendResult:
    ok: bool
    message: str


class MockFrankyBackend:
    """Offline backend: logs intended motion, no hardware."""

    def __init__(self) -> None:
        self._connected = False
        self._fault_active = False

    def connect(self) -> BackendResult:
        self._connected = True
        return BackendResult(True, 'backend connected (mock)')

    def execute_arm_action(self, msg) -> BackendResult:
        if not self._connected:
            return BackendResult(False, 'backend not connected')
        if self._fault_active:
            return BackendResult(False, 'backend fault active; call /reset_arm_fault')
        if msg.control_mode == ARM_JOINT_POSITION:
            return BackendResult(True, f'joint_position target={_fmt_vec(list(msg.joint_position))}')
        if msg.control_mode == ARM_JOINT_VELOCITY:
            return BackendResult(True, f'joint_velocity target={_fmt_vec(list(msg.joint_velocity))}')
        if msg.control_mode == ARM_CARTESIAN_POSE:
            p = msg.cartesian_pose.position
            return BackendResult(True, f'cartesian_pose target=({p.x:.4f}, {p.y:.4f}, {p.z:.4f})')
        if msg.control_mode == ARM_CARTESIAN_VELOCITY:
            lv = msg.cartesian_velocity.linear
            av = msg.cartesian_velocity.angular
            return BackendResult(
                True,
                (
                    'cartesian_velocity '
                    f'lin=({lv.x:.4f}, {lv.y:.4f}, {lv.z:.4f}) '
                    f'ang=({av.x:.4f}, {av.y:.4f}, {av.z:.4f})'
                ),
            )
        return BackendResult(False, f'unsupported control_mode={msg.control_mode}')

    def stop(self, immediate: bool) -> BackendResult:
        if not self._connected:
            return BackendResult(False, 'backend not connected')
        return BackendResult(True, f'stop accepted (immediate={immediate})')

    def home(self, wait: bool, timeout_sec: float) -> BackendResult:
        if not self._connected:
            return BackendResult(False, 'backend not connected')
        return BackendResult(True, f'home accepted (wait={wait}, timeout={timeout_sec:.2f}s)')

    def reset_fault(self, hard_reset: bool, timeout_sec: float) -> BackendResult:
        if not self._connected:
            return BackendResult(False, 'backend not connected')
        self._fault_active = False
        return BackendResult(
            True, f'reset_fault accepted (hard_reset={hard_reset}, timeout={timeout_sec:.2f}s)'
        )


def drain_event_queue(q: queue.Queue) -> List[Tuple[str, str]]:
    out: List[Tuple[str, str]] = []
    while True:
        try:
            out.append(q.get_nowait())
        except queue.Empty:
            break
    return out
