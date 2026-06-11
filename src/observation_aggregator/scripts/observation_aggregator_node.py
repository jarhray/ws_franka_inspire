#!/usr/bin/env python3

import math
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, TwistStamped
from robot_interfaces.msg import RobotObservation
from sensor_msgs.msg import CameraInfo, Image, JointState

from service_interfaces.msg import GetAngleAct1, GetTouchAct1

# ---------------------------------------------------------------------------
# Inspire 手指标定：硬件寄存器 k∈[0,1000]（整数）→ 关节弧度 r = f(k)。
# 必须与 inspire_executor/inspire_executor_node.py 中 radians_from_hardware_* 完全一致，
# 以便策略层观测（弧度）与执行器（r→k 逆映射）同一套物理含义。
# finger_id：0–3 四指弯曲，4 拇指弯曲，5 拇指侧摆。
# ---------------------------------------------------------------------------


def radians_from_hardware_four_fingers(k: float) -> float:
    """Pinky / Ring / Middle / Index：r = f(k)。"""
    return -5e-10 * k**3 + 9e-7 * k**2 - 0.0018 * k + 1.4191


def radians_from_hardware_thumb_flexion(k: float) -> float:
    """Thumb flexion (id=4)。"""
    return 8e-11 * k**3 - 4e-8 * k**2 - 0.0006 * k + 0.5869


def radians_from_hardware_thumb_abduction(k: float) -> float:
    """Thumb abduction (id=5)，线性。"""
    return -0.0012 * k + 1.1641


def radians_from_hardware_for_finger_id(k: float, finger_id: int) -> float:
    if finger_id in (0, 1, 2, 3):
        return radians_from_hardware_four_fingers(k)
    if finger_id == 4:
        return radians_from_hardware_thumb_flexion(k)
    if finger_id == 5:
        return radians_from_hardware_thumb_abduction(k)
    return radians_from_hardware_four_fingers(k)


def _qualify(namespace: str, topic: str) -> str:
    """Join namespace + topic path with correct slashes."""
    if not topic.startswith("/"):
        topic = "/" + topic
    if namespace is None:
        namespace = ""
    namespace = namespace.strip("/")
    if namespace == "":
        return topic
    return f"/{namespace}{topic}"


def _time_from_msg(node: Node, stamp_msg) -> Optional[Time]:
    try:
        return Time.from_msg(stamp_msg, clock_type=node.get_clock().clock_type)
    except Exception:
        return None


class ObservationAggregator(Node):
    def __init__(self) -> None:
        super().__init__("observation_aggregator")

        # ---- Parameters (RealSense) ----
        self.rs_rgb_image_topic = self.declare_parameter(
            "rs_rgb_image_topic", "/camera/color/image_raw"
        ).value
        self.rs_depth_image_topic = self.declare_parameter(
            "rs_depth_image_topic", "/camera/depth/image_rect_raw"
        ).value
        self.rs_camera_info_topic = self.declare_parameter(
            "rs_camera_info_topic", "/camera/color/camera_info"
        ).value
        # If your RealSense node is nested (e.g. /camera/camera/color/image_raw), either
        # set the three rs_* params above, or add alternates here (all are subscribed).
        self.rs_rgb_image_topic_alternates = self.declare_parameter(
            "rs_rgb_image_topic_alternates",
            ["/camera/camera/color/image_raw"],
        ).value
        self.rs_depth_image_topic_alternates = self.declare_parameter(
            "rs_depth_image_topic_alternates",
            ["/camera/camera/depth/image_rect_raw"],
        ).value
        self.rs_camera_info_topic_alternates = self.declare_parameter(
            "rs_camera_info_topic_alternates",
            ["/camera/camera/color/camera_info"],
        ).value

        # ---- Parameters (multi-camera) ----
        # ``enabled_cameras``: comma-separated ids matching hardware_bringup.launch.py
        # (e.g. cam1,cam3). When non-empty, overrides observation_camera_ids and auto-subscribes to
        # /{id}/{id}/color/image_raw, /{id}/{id}/depth/image_rect_raw, /{id}/{id}/color/camera_info
        # (same camera_name + camera_namespace layout as rs_launch for each RealSense).
        self.enabled_cameras = str(
            self.declare_parameter("enabled_cameras", "").value or ""
        ).strip()

        self.observation_camera_ids = list(
            self.declare_parameter("observation_camera_ids", []).value or []
        )
        self.observation_camera_rgb_topics = list(
            self.declare_parameter("observation_camera_rgb_topics", []).value or []
        )
        self.observation_camera_depth_topics = list(
            self.declare_parameter("observation_camera_depth_topics", []).value or []
        )
        self.observation_camera_info_topics = list(
            self.declare_parameter("observation_camera_info_topics", []).value or []
        )
        self.default_camera_id = str(
            self.declare_parameter("default_camera_id", "cam1").value or "cam1"
        )

        if self.enabled_cameras:
            self.observation_camera_ids = [
                x.strip() for x in self.enabled_cameras.split(",") if x.strip()
            ]
            if not self.observation_camera_ids:
                raise ValueError("enabled_cameras is non-empty but parses to no camera ids")
            self.observation_camera_rgb_topics = []
            self.observation_camera_depth_topics = []
            self.observation_camera_info_topics = []

        n_ids = len(self.observation_camera_ids)
        auto_topics = False
        if n_ids > 0:
            n_rgb = len(self.observation_camera_rgb_topics)
            n_dep = len(self.observation_camera_depth_topics)
            auto_topics = n_rgb == 0 and n_dep == 0
            if auto_topics:
                for cid in self.observation_camera_ids:
                    c = str(cid).strip()
                    self.observation_camera_rgb_topics.append(f"/{c}/{c}/color/image_raw")
                    self.observation_camera_depth_topics.append(
                        f"/{c}/{c}/depth/image_rect_raw"
                    )
                n_info = len(self.observation_camera_info_topics)
                if n_info == 0:
                    for cid in self.observation_camera_ids:
                        c = str(cid).strip()
                        self.observation_camera_info_topics.append(
                            f"/{c}/{c}/color/camera_info"
                        )
                elif n_info != n_ids:
                    raise ValueError(
                        "With auto topic layout, observation_camera_info_topics must be "
                        "empty (use default color camera_info per camera) or same length as "
                        "observation_camera_ids"
                    )
            elif n_rgb != n_ids or n_dep != n_ids:
                raise ValueError(
                    "observation_camera_ids length must match "
                    "observation_camera_rgb_topics and observation_camera_depth_topics "
                    "(or leave rgb/depth topic lists empty for auto layout)"
                )
            if len(self.observation_camera_info_topics) == 0:
                self.observation_camera_info_topics = [""] * n_ids
            elif len(self.observation_camera_info_topics) != n_ids:
                raise ValueError(
                    "observation_camera_info_topics must be empty (skip all info) or "
                    "same length as observation_camera_ids"
                )

        self._multi_camera_mode = n_ids > 0
        if self._multi_camera_mode and self.enabled_cameras:
            self.get_logger().info(
                f"enabled_cameras={self.enabled_cameras!r} -> topics aligned with "
                f"hardware_bringup /{{id}}/{{id}}/..."
            )
        elif self._multi_camera_mode and auto_topics:
            self.get_logger().info(
                "Multi-camera: auto RealSense topics /{id}/{id}/color|depth|camera_info "
                f"for ids={self.observation_camera_ids!r}"
            )

        # Per-camera caches (multi mode)
        self._mc_order: List[str] = list(self.observation_camera_ids)
        self._mc_rgb: Dict[str, Optional[Image]] = {c: None for c in self._mc_order}
        self._mc_depth: Dict[str, Optional[Image]] = {c: None for c in self._mc_order}
        self._mc_info: Dict[str, Optional[CameraInfo]] = {c: None for c in self._mc_order}
        self._mc_stamp: Dict[str, Optional[Time]] = {c: None for c in self._mc_order}
        self._warned_bad_default_cam = False

        # ---- Parameters (Inspire) ----
        self.hand_angle_topic = self.declare_parameter(
            "hand_angle_topic", "/angle_data"
        ).value
        self.hand_finger_id_order = self.declare_parameter(
            "hand_finger_id_order", [0, 1, 2, 3, 4, 5]
        ).value
        self.hand_touch_topic = self.declare_parameter(
            "hand_touch_topic", "/touch_data"
        ).value

        # ---- Parameters (FR3) ----
        self.observation_topic = self.declare_parameter(
            "observation_topic", "/robot/observation"
        ).value
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 30.0).value
        )
        self.arm_joint_name_order = self.declare_parameter(
            "arm_joint_name_order", []
        ).value

        # franka.launch.py default spawns:
        # - ros2_control_node (controller_manager)
        # - controllers:
        #   - joint_state_broadcaster
        #   - franka_robot_state_broadcaster
        # Those controllers publish "private" topics (~/...). We subscribe to a few likely candidates.
        self.franka_namespace = self.declare_parameter("franka_namespace", "").value
        self.franka_state_controller_name = self.declare_parameter(
            "franka_state_controller_name", "franka_robot_state_broadcaster"
        ).value
        self.ros2_control_node_name = self.declare_parameter(
            "ros2_control_node_name", "ros2_control_node"
        ).value

        # When use_fake_hardware:=true, franka.launch.py does NOT spawn
        # franka_robot_state_broadcaster; only joint_state_broadcaster feeds
        # /franka/joint_states and /joint_states (see franka_bringup franka.launch.py).
        self.arm_joint_state_extra_topics = self.declare_parameter(
            "arm_joint_state_extra_topics",
            ["/franka/joint_states", "/joint_states"],
        ).value

        # Candidate topic bases (in priority order).
        self.arm_joint_state_topic_candidates = [
            _qualify(
                self.franka_namespace,
                f"/{self.franka_state_controller_name}/measured_joint_states",
            ),
            _qualify(
                self.franka_namespace,
                f"/{self.ros2_control_node_name}/measured_joint_states",
            ),
            _qualify(self.franka_namespace, "/measured_joint_states"),
        ]
        for t in self.arm_joint_state_extra_topics:
            if not t:
                continue
            tt = t if str(t).startswith("/") else "/" + str(t)
            if tt not in self.arm_joint_state_topic_candidates:
                self.arm_joint_state_topic_candidates.append(tt)
        self.arm_pose_topic_candidates = [
            _qualify(self.franka_namespace, f"/{self.franka_state_controller_name}/current_pose"),
            _qualify(self.franka_namespace, f"/{self.ros2_control_node_name}/current_pose"),
            _qualify(self.franka_namespace, "/current_pose"),
        ]
        self.arm_twist_topic_candidates = [
            _qualify(
                self.franka_namespace,
                f"/{self.franka_state_controller_name}/desired_end_effector_twist",
            ),
            _qualify(
                self.franka_namespace,
                f"/{self.ros2_control_node_name}/desired_end_effector_twist",
            ),
            _qualify(self.franka_namespace, "/desired_end_effector_twist"),
        ]

        # ROS 2 Humble: use qos_profile_sensor_data (SensorDataQoS exists in newer APIs only).
        qos = qos_profile_sensor_data

        # ---- Subscribers (FR3) ----
        self.last_arm_joint_state: Optional[JointState] = None
        self.has_arm_pose = False
        self.has_arm_twist = False

        self.last_arm_joint_stamp: Optional[Time] = None
        self.last_arm_pose: Optional[PoseStamped] = None
        self.last_arm_pose_stamp: Optional[Time] = None
        self.last_arm_twist: Optional[TwistStamped] = None
        self.last_arm_twist_stamp: Optional[Time] = None

        self._arm_joint_subs = []
        for t in self.arm_joint_state_topic_candidates:
            self._arm_joint_subs.append(
                self.create_subscription(JointState, t, self._on_arm_joint, qos)
            )

        self._arm_pose_subs = []
        for t in self.arm_pose_topic_candidates:
            self._arm_pose_subs.append(
                self.create_subscription(PoseStamped, t, self._on_arm_pose, qos)
            )

        self._arm_twist_subs = []
        for t in self.arm_twist_topic_candidates:
            self._arm_twist_subs.append(
                self.create_subscription(
                    TwistStamped, t, self._on_arm_twist, qos
                )
            )

        # ---- Subscribers (Hand) ----
        # 回调里只缓存 angle_data 的原始硬件整数 k（与原先一致）；弧度仅在发布 RobotObservation 时换算。
        self._hand_k_ordered: Optional[List[float]] = None
        self._hand_vel_radians: List[float] = []
        self.last_hand_time: Optional[Time] = None
        self._last_hand_has_value = False

        self.create_subscription(GetAngleAct1, self.hand_angle_topic, self._on_hand_angle, qos)

        self.last_touch_msg: Optional[GetTouchAct1] = None
        self.last_touch_time: Optional[Time] = None
        self._last_touch_has_value = False
        self.create_subscription(GetTouchAct1, self.hand_touch_topic, self._on_hand_touch, qos)

        # ---- Subscribers (RealSense) ----
        self.last_rgb_image: Optional[Image] = None
        self.last_depth_image: Optional[Image] = None
        self.last_camera_info: Optional[CameraInfo] = None
        self.last_rs_stamp: Optional[Time] = None

        if self._multi_camera_mode:
            for i, cam_id in enumerate(self._mc_order):
                cid = str(cam_id)
                t_rgb = str(self.observation_camera_rgb_topics[i]).strip()
                t_depth = str(self.observation_camera_depth_topics[i]).strip()
                t_info = str(self.observation_camera_info_topics[i]).strip()
                if not t_rgb or not t_depth:
                    raise ValueError(
                        f"Multi-camera {cid}: rgb/depth topic must be non-empty "
                        f"(got rgb={t_rgb!r} depth={t_depth!r})"
                    )
                self.create_subscription(
                    Image, t_rgb, self._make_mc_rgb_cb(cid), qos
                )
                self.create_subscription(
                    Image, t_depth, self._make_mc_depth_cb(cid), qos
                )
                if t_info:
                    self.create_subscription(
                        CameraInfo, t_info, self._make_mc_info_cb(cid), qos
                    )
            self.get_logger().info(
                f"Multi-camera mode: {self._mc_order} default_legacy_mirror={self.default_camera_id!r}"
            )
        else:
            self.create_subscription(Image, self.rs_rgb_image_topic, self._on_rgb_image, qos)
            self.create_subscription(Image, self.rs_depth_image_topic, self._on_depth_image, qos)
            self.create_subscription(
                CameraInfo, self.rs_camera_info_topic, self._on_camera_info, qos
            )
            for alt in self.rs_rgb_image_topic_alternates:
                if alt and alt != self.rs_rgb_image_topic:
                    self.create_subscription(Image, alt, self._on_rgb_image, qos)
            for alt in self.rs_depth_image_topic_alternates:
                if alt and alt != self.rs_depth_image_topic:
                    self.create_subscription(Image, alt, self._on_depth_image, qos)
            for alt in self.rs_camera_info_topic_alternates:
                if alt and alt != self.rs_camera_info_topic:
                    self.create_subscription(CameraInfo, alt, self._on_camera_info, qos)

        # ---- Publisher ----
        self.observation_pub = self.create_publisher(RobotObservation, self.observation_topic, 10)

        self.timer = self.create_timer(1.0 / max(0.1, self.publish_rate_hz), self._on_timer)

        self.get_logger().info("observation_aggregator (Python) started")
        self.get_logger().info(f"Publishing: {self.observation_topic}")
        self.get_logger().info(
            "FR3 JointState subscription candidates (first match wins per callback): "
            f"{self.arm_joint_state_topic_candidates}"
        )
        self.get_logger().info(
            "Note: with use_fake_hardware:=true, franka_robot_state_broadcaster is not loaded; "
            "use /franka/joint_states or /joint_states (see arm_joint_state_extra_topics)."
        )
        if not self._multi_camera_mode:
            self.get_logger().info(
                f"RealSense RGB: {self.rs_rgb_image_topic} "
                f"(+ alternates {self.rs_rgb_image_topic_alternates})"
            )
        self.get_logger().info(f"Inspire touch: {self.hand_touch_topic}")

    def _merge_rs_stamp(self, t: Optional[Time]) -> None:
        if t is None:
            return
        if self.last_rs_stamp is None or t.nanoseconds > self.last_rs_stamp.nanoseconds:
            self.last_rs_stamp = t

    def _make_mc_rgb_cb(self, cam_id: str):
        def _cb(msg: Image) -> None:
            self._mc_rgb[cam_id] = msg
            tt = _time_from_msg(self, msg.header.stamp)
            self._mc_stamp[cam_id] = tt
            self._merge_rs_stamp(tt)

        return _cb

    def _make_mc_depth_cb(self, cam_id: str):
        def _cb(msg: Image) -> None:
            self._mc_depth[cam_id] = msg
            tt = _time_from_msg(self, msg.header.stamp)
            self._mc_stamp[cam_id] = tt
            self._merge_rs_stamp(tt)

        return _cb

    def _make_mc_info_cb(self, cam_id: str):
        def _cb(msg: CameraInfo) -> None:
            self._mc_info[cam_id] = msg
            tt = _time_from_msg(self, msg.header.stamp)
            self._mc_stamp[cam_id] = tt
            self._merge_rs_stamp(tt)

        return _cb

    # ----------------- Callbacks -----------------
    def _on_arm_joint(self, msg: JointState) -> None:
        self.last_arm_joint_state = msg
        self.last_arm_joint_stamp = _time_from_msg(self, msg.header.stamp)

    def _on_arm_pose(self, msg: PoseStamped) -> None:
        self.last_arm_pose = msg
        self.last_arm_pose_stamp = _time_from_msg(self, msg.header.stamp)
        self.has_arm_pose = True

    def _on_arm_twist(self, msg: TwistStamped) -> None:
        self.last_arm_twist = msg
        self.last_arm_twist_stamp = _time_from_msg(self, msg.header.stamp)
        self.has_arm_twist = True

    def _on_hand_angle(self, msg: GetAngleAct1) -> None:
        now = self.get_clock().now()

        # finger_id -> hardware k（与原先一致：直接来自 GetAngleAct1.angles）
        angle_by_id: Dict[int, float] = {}
        n = min(len(msg.finger_ids), len(msg.angles))
        for i in range(n):
            angle_by_id[int(msg.finger_ids[i])] = float(msg.angles[i])

        k_ordered: List[float] = [math.nan for _ in self.hand_finger_id_order]
        for j, fid in enumerate(self.hand_finger_id_order):
            fid_i = int(fid)
            if fid_i in angle_by_id:
                k_ordered[j] = angle_by_id[fid_i]

        # 角速度（rad/s）：对 r=f(k) 做差分，与发布侧同一套 f(k)
        n_j = len(self.hand_finger_id_order)
        vel_r = [0.0 for _ in range(n_j)]
        if (
            self._hand_k_ordered is not None
            and self.last_hand_time is not None
            and len(self._hand_k_ordered) == n_j
        ):
            dt = (now - self.last_hand_time).nanoseconds / 1e9
            if dt > 1e-6:
                for j in range(n_j):
                    fid_i = int(self.hand_finger_id_order[j])
                    k_new = k_ordered[j]
                    k_old = self._hand_k_ordered[j]
                    if math.isfinite(k_new) and math.isfinite(k_old):
                        kn = max(0.0, min(1000.0, k_new))
                        ko = max(0.0, min(1000.0, k_old))
                        r_new = radians_from_hardware_for_finger_id(kn, fid_i)
                        r_old = radians_from_hardware_for_finger_id(ko, fid_i)
                        vel_r[j] = (r_new - r_old) / dt

        self._hand_k_ordered = k_ordered
        self._hand_vel_radians = vel_r
        self.last_hand_time = now
        self._last_hand_has_value = True

    def _on_hand_touch(self, msg: GetTouchAct1) -> None:
        self.last_touch_msg = msg
        self.last_touch_time = self.get_clock().now()
        self._last_touch_has_value = True

    def _on_rgb_image(self, msg: Image) -> None:
        self.last_rgb_image = msg
        self._merge_rs_stamp(_time_from_msg(self, msg.header.stamp))

    def _on_depth_image(self, msg: Image) -> None:
        self.last_depth_image = msg
        self._merge_rs_stamp(_time_from_msg(self, msg.header.stamp))

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self.last_camera_info = msg
        self._merge_rs_stamp(_time_from_msg(self, msg.header.stamp))

    # ----------------- Helpers -----------------
    def _build_arm_positions(self) -> List[float]:
        if self.last_arm_joint_state is None:
            return []
        js = self.last_arm_joint_state

        # Default: keep JointState order.
        if not self.arm_joint_name_order:
            return list(js.position)

        pos_by_name = {js.name[i]: js.position[i] for i in range(min(len(js.name), len(js.position)))}
        return [pos_by_name.get(n, 0.0) for n in self.arm_joint_name_order]

    def _build_arm_velocities(self) -> List[float]:
        if self.last_arm_joint_state is None:
            return []
        js = self.last_arm_joint_state
        has_velocity = len(js.velocity) > 0

        if not self.arm_joint_name_order:
            if not has_velocity:
                return [0.0 for _ in js.position]
            return list(js.velocity)

        vel_by_name = {
            js.name[i]: js.velocity[i]
            for i in range(min(len(js.name), len(js.velocity)))
        }
        # If velocity missing some joints, default to 0.0
        if not has_velocity:
            return [0.0 for _ in self.arm_joint_name_order]
        return [vel_by_name.get(n, 0.0) for n in self.arm_joint_name_order]

    def _hand_k_ordered_to_radians(self) -> Tuple[List[float], List[float]]:
        """由缓存的硬件 k 得到 hand_joint_position（弧度）与 hand_joint_velocity（rad/s）。"""
        if self._hand_k_ordered is None:
            return [], []
        pos: List[float] = []
        for j, fid in enumerate(self.hand_finger_id_order):
            k = self._hand_k_ordered[j] if j < len(self._hand_k_ordered) else math.nan
            if not math.isfinite(k):
                pos.append(math.nan)
            else:
                kc = max(0.0, min(1000.0, k))
                pos.append(radians_from_hardware_for_finger_id(kc, int(fid)))
        return pos, list(self._hand_vel_radians)

    @staticmethod
    def _max_time(times: List[Optional[Time]]) -> Optional[Time]:
        valid = [t for t in times if t is not None]
        if not valid:
            return None
        return max(valid, key=lambda x: x.nanoseconds)

    def _fill_hand_touch(self, obs: RobotObservation) -> None:
        if not self._last_touch_has_value or self.last_touch_msg is None:
            return
        src = self.last_touch_msg
        obs.hand_touch.finger_ids = list(src.finger_ids)
        obs.hand_touch.finger_names = list(src.finger_names)
        obs.hand_touch.normal_forces = list(src.normal_forces)
        obs.hand_touch.tangential_forces = list(src.tangential_forces)

    # ----------------- Timer -----------------
    def _on_timer(self) -> None:
        obs = RobotObservation()

        stamp = self._max_time(
            [
                self.last_arm_joint_stamp,
                self.last_arm_pose_stamp,
                self.last_arm_twist_stamp,
                self.last_hand_time,
                self.last_touch_time,
                self.last_rs_stamp,
            ]
        )
        if stamp is None:
            stamp = self.get_clock().now()
        obs.header.stamp = stamp.to_msg()

        obs.arm_joint_position = self._build_arm_positions()
        obs.arm_joint_velocity = self._build_arm_velocities()

        if self.has_arm_pose and self.last_arm_pose is not None:
            obs.ee_pose = self.last_arm_pose.pose
            obs.ee_pose_frame = self.last_arm_pose.header.frame_id
        if self.has_arm_twist and self.last_arm_twist is not None:
            obs.ee_twist = self.last_arm_twist.twist

        if self._last_hand_has_value and self._hand_k_ordered is not None:
            hp, hv = self._hand_k_ordered_to_radians()
            obs.hand_joint_position = hp
            obs.hand_joint_velocity = hv

        self._fill_hand_touch(obs)

        if self._multi_camera_mode:
            obs.camera_ids = list(self._mc_order)
            obs.rgb_images = []
            obs.depth_images = []
            obs.camera_infos = []
            for cid in self._mc_order:
                r = self._mc_rgb.get(cid)
                d = self._mc_depth.get(cid)
                info = self._mc_info.get(cid)
                obs.rgb_images.append(r if r is not None else Image())
                obs.depth_images.append(d if d is not None else Image())
                obs.camera_infos.append(info if info is not None else CameraInfo())

            primary = self.default_camera_id.strip()
            if primary not in self._mc_order:
                primary = self._mc_order[0]
                if not self._warned_bad_default_cam:
                    self.get_logger().warn(
                        f"default_camera_id={self.default_camera_id!r} not in {self._mc_order}; "
                        f"using {primary!r} for legacy rgb_image/depth_image/camera_info mirror."
                    )
                    self._warned_bad_default_cam = True
            pidx = self._mc_order.index(primary)
            if pidx < len(obs.rgb_images) and obs.rgb_images[pidx].height > 0:
                obs.rgb_image = obs.rgb_images[pidx]
            if pidx < len(obs.depth_images) and obs.depth_images[pidx].height > 0:
                obs.depth_image = obs.depth_images[pidx]
            if pidx < len(obs.camera_infos):
                obs.camera_info = obs.camera_infos[pidx]
        else:
            if self.last_rgb_image is not None:
                obs.rgb_image = self.last_rgb_image
            if self.last_depth_image is not None:
                obs.depth_image = self.last_depth_image
            if self.last_camera_info is not None:
                obs.camera_info = self.last_camera_info

        self.observation_pub.publish(obs)


def main() -> None:
    rclpy.init()
    node = ObservationAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

