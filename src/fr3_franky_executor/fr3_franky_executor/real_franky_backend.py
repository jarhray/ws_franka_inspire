"""Real robot backend via franky.Robot (optional dependency for mock-only users).

franky is imported lazily: ``ros2 run`` typically uses ``/usr/bin/python3``, while
``franky-control`` is often installed only in a venv — see README for PYTHONPATH / pip fix.
"""

from __future__ import annotations

import sys
import threading
from dataclasses import dataclass, field
from typing import Any, List, Optional

import numpy as np

from robot_interfaces.msg import ArmAction

from .franky_backend_impl import (
    ARM_CARTESIAN_POSE,
    ARM_CARTESIAN_VELOCITY,
    ARM_JOINT_POSITION,
    ARM_JOINT_VELOCITY,
    BackendResult,
    _duration_from_sec,
    _fmt_vec,
    _finite,
    _joint_array7,
    _pose_to_affine,
)


def _parse_controller_mode(name: str, franky_mod: Any) -> Any:
    ControllerMode = franky_mod.ControllerMode
    n = (name or '').strip().lower()
    if n in ('cartesian', 'cartesian_impedance', 'cart'):
        return ControllerMode.CartesianImpedance
    return ControllerMode.JointImpedance


def _franky_import_error_message() -> str:
    return (
        f'无法 import franky（PyPI 包名: franky-control）。当前进程 Python: {sys.executable}。'
        f'请用同一解释器安装: `{sys.executable} -m pip install franky-control`，'
        f'或设置 PYTHONPATH 指向已安装 franky 的 site-packages（例如 .venv）。'
    )


@dataclass(frozen=True)
class FrankyRobotState:
    joint_position: List[float]
    joint_velocity: List[float]
    ee_translation: List[float]
    ee_quaternion: List[float]
    ee_linear_velocity: List[float]
    ee_angular_velocity: List[float]


def _as_float_list(v: Any) -> List[float]:
    return [float(x) for x in np.asarray(v).reshape(-1).tolist()]


@dataclass
class RealFrankyBackend:
    """Uses franky.Robot.move() for FR3 / Franka FCI."""

    fci_hostname: str
    controller_mode: str
    relative_dynamics_factor: float
    default_velocity_duration_sec: float
    home_joint_position: List[float]
    velocity_async: bool
    position_async: bool = True

    _robot: Optional[Any] = field(default=None, init=False, repr=False)
    _fault_active: bool = field(default=False, init=False, repr=False)
    _franky_mod: Optional[Any] = field(default=None, init=False, repr=False)
    _controller_mode_enum: Any = field(default=None, init=False, repr=False)
    _rel_dyn: float = field(default=1.0, init=False, repr=False)
    _default_velocity_duration_sec_f: float = field(default=0.1, init=False, repr=False)
    _home_joint_position_l: List[float] = field(default_factory=list, init=False, repr=False)
    _velocity_async_b: bool = field(default=True, init=False, repr=False)
    _position_async_b: bool = field(default=True, init=False, repr=False)
    _robot_lock: threading.RLock = field(default_factory=threading.RLock, init=False, repr=False)

    def __post_init__(self) -> None:
        self._rel_dyn = float(self.relative_dynamics_factor)
        self._default_velocity_duration_sec_f = float(self.default_velocity_duration_sec)
        self._home_joint_position_l = list(self.home_joint_position)
        self._velocity_async_b = bool(self.velocity_async)
        self._position_async_b = bool(self.position_async)

    def _ensure_franky_mod(self) -> Optional[Any]:
        if self._franky_mod is not None:
            return self._franky_mod
        try:
            import franky

            self._franky_mod = franky
            if self._controller_mode_enum is None:
                self._controller_mode_enum = _parse_controller_mode(self.controller_mode, franky)
            return franky
        except ImportError:
            return None

    @property
    def robot(self) -> Optional[Any]:
        return self._robot

    def connect(self) -> BackendResult:
        franky = self._ensure_franky_mod()
        if franky is None:
            return BackendResult(False, _franky_import_error_message())

        host = (self.fci_hostname or '').strip()
        if not host:
            return BackendResult(False, 'fci_hostname is empty; set fr3_franky_executor/fci_hostname')

        try:
            with self._robot_lock:
                self._robot = franky.Robot(
                    host,
                    relative_dynamics_factor=self._rel_dyn,
                    controller_mode=self._controller_mode_enum,
                )
        except Exception as e:
            with self._robot_lock:
                self._robot = None
            return BackendResult(False, f'Robot() failed: {e!s}')

        return BackendResult(True, f'backend connected to FCI at {host!r} (franky)')

    def _ensure_robot(self) -> Any:
        if self._robot is None:
            raise RuntimeError('robot not connected')
        return self._robot

    def execute_arm_action(self, msg: ArmAction) -> BackendResult:
        if self._ensure_franky_mod() is None:
            return BackendResult(False, _franky_import_error_message())

        with self._robot_lock:
            robot = self._ensure_robot()
            franky = self._franky_mod
            assert franky is not None

            if self._fault_active:
                return BackendResult(False, 'backend fault active; call /reset_arm_fault')
            if robot.has_errors:
                return BackendResult(False, 'robot.has_errors is true; call /reset_arm_fault')

            ReferenceType = franky.ReferenceType
            ref = ReferenceType.Relative if msg.is_relative else ReferenceType.Absolute

            try:
                if msg.control_mode == ARM_JOINT_POSITION:
                    target = _joint_array7(list(msg.joint_position))
                    motion = franky.JointMotion(
                        target,
                        reference_type=ref,
                        relative_dynamics_factor=self._rel_dyn,
                        return_when_finished=True,
                    )
                    robot.move(motion, asynchronous=self._position_async_b)
                    return BackendResult(
                        True,
                        f'joint_position ok target={_fmt_vec(list(msg.joint_position))} async={self._position_async_b}',
                    )

                if msg.control_mode == ARM_JOINT_VELOCITY:
                    target = _joint_array7(list(msg.joint_velocity))
                    dur_sec = (
                        float(msg.duration_sec)
                        if _finite(float(msg.duration_sec))
                        else self._default_velocity_duration_sec_f
                    )
                    if dur_sec <= 0.0:
                        dur_sec = self._default_velocity_duration_sec_f
                    motion = franky.JointVelocityMotion(
                        target,
                        duration=_duration_from_sec(dur_sec),
                        relative_dynamics_factor=self._rel_dyn,
                    )
                    robot.move(motion, asynchronous=self._velocity_async_b)
                    return BackendResult(True, f'joint_velocity ok dur={dur_sec:.3f}s async={self._velocity_async_b}')

                if msg.control_mode == ARM_CARTESIAN_POSE:
                    affine = _pose_to_affine(msg.cartesian_pose)
                    motion = franky.CartesianMotion(
                        affine,
                        reference_type=ref,
                        relative_dynamics_factor=self._rel_dyn,
                        return_when_finished=True,
                    )
                    robot.move(motion, asynchronous=self._position_async_b)
                    p = msg.cartesian_pose.position
                    return BackendResult(
                        True,
                        f'cartesian_pose ok pos=({p.x:.4f},{p.y:.4f},{p.z:.4f}) async={self._position_async_b}',
                    )

                if msg.control_mode == ARM_CARTESIAN_VELOCITY:
                    lv = msg.cartesian_velocity.linear
                    av = msg.cartesian_velocity.angular
                    lin = np.array([[lv.x], [lv.y], [lv.z]], dtype=np.float64)
                    ang = np.array([[av.x], [av.y], [av.z]], dtype=np.float64)
                    tw = franky.Twist(linear_velocity=lin, angular_velocity=ang)
                    dur_sec = (
                        float(msg.duration_sec)
                        if _finite(float(msg.duration_sec))
                        else self._default_velocity_duration_sec_f
                    )
                    if dur_sec <= 0.0:
                        dur_sec = self._default_velocity_duration_sec_f
                    motion = franky.CartesianVelocityMotion(
                        tw,
                        duration=_duration_from_sec(dur_sec),
                        relative_dynamics_factor=self._rel_dyn,
                    )
                    robot.move(motion, asynchronous=self._velocity_async_b)
                    return BackendResult(
                        True, f'cartesian_velocity ok dur={dur_sec:.3f}s async={self._velocity_async_b}'
                    )

                return BackendResult(False, f'unsupported control_mode={msg.control_mode}')
            except Exception as e:
                return BackendResult(False, f'franky move failed: {e!s}')

    def stop(self, immediate: bool) -> BackendResult:
        del immediate
        if self._ensure_franky_mod() is None:
            return BackendResult(False, _franky_import_error_message())
        try:
            with self._robot_lock:
                robot = self._ensure_robot()
                robot.stop()
            return BackendResult(True, 'franky Robot.stop() called')
        except Exception as e:
            return BackendResult(False, f'stop failed: {e!s}')

    def home(self, wait: bool, timeout_sec: float) -> BackendResult:
        del wait, timeout_sec
        if self._ensure_franky_mod() is None:
            return BackendResult(False, _franky_import_error_message())
        if len(self._home_joint_position_l) != 7:
            return BackendResult(False, 'home_joint_position must have length 7')
        try:
            with self._robot_lock:
                franky = self._franky_mod
                assert franky is not None
                robot = self._ensure_robot()
                target = _joint_array7(self._home_joint_position_l)
                motion = franky.JointMotion(
                    target,
                    reference_type=franky.ReferenceType.Absolute,
                    relative_dynamics_factor=self._rel_dyn,
                    return_when_finished=True,
                )
                robot.move(motion, asynchronous=False)
            return BackendResult(True, 'home motion finished (JointMotion to home_joint_position)')
        except Exception as e:
            return BackendResult(False, f'home failed: {e!s}')

    def reset_fault(self, hard_reset: bool, timeout_sec: float) -> BackendResult:
        del timeout_sec
        if self._ensure_franky_mod() is None:
            return BackendResult(False, _franky_import_error_message())
        try:
            with self._robot_lock:
                robot = self._ensure_robot()
                ok = bool(robot.recover_from_errors())
                if hard_reset:
                    ok = bool(robot.recover_from_errors()) and ok
                self._fault_active = False
            if ok:
                return BackendResult(True, 'recover_from_errors() succeeded')
            return BackendResult(False, 'recover_from_errors() returned false')
        except Exception as e:
            return BackendResult(False, f'recover_from_errors failed: {e!s}')

    def read_state(self) -> FrankyRobotState:
        """Read current robot state for ROS topic publication."""
        if self._ensure_franky_mod() is None:
            raise RuntimeError(_franky_import_error_message())

        with self._robot_lock:
            robot = self._ensure_robot()
            q = _as_float_list(robot.current_joint_positions)
            dq = _as_float_list(robot.current_joint_velocities)

            cstate = robot.current_cartesian_state
            pose_aff = cstate.pose.end_effector_pose
            quat = _as_float_list(np.asarray(pose_aff.quaternion))
            trans = _as_float_list(np.asarray(pose_aff.translation))

            cvel = robot.current_cartesian_velocity
            tw = cvel.end_effector_twist
            lin = _as_float_list(np.asarray(tw.linear))
            ang = _as_float_list(np.asarray(tw.angular))

        return FrankyRobotState(
            joint_position=q,
            joint_velocity=dq,
            ee_translation=trans,
            ee_quaternion=quat,
            ee_linear_velocity=lin,
            ee_angular_velocity=ang,
        )

    def motion_in_progress(self) -> bool:
        """Best-effort check whether robot has an active asynchronous motion."""
        if self._ensure_franky_mod() is None:
            return False
        with self._robot_lock:
            robot = self._ensure_robot()
            for attr in ('is_moving', 'motion_running', 'is_motion_running', 'is_busy'):
                v = getattr(robot, attr, None)
                if callable(v):
                    try:
                        return bool(v())
                    except Exception:
                        continue
                if v is not None:
                    try:
                        return bool(v)
                    except Exception:
                        continue
        return False
