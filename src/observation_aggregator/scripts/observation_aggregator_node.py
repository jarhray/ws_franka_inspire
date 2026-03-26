#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import SensorDataQoS
from rclpy.time import Time

from builtin_interfaces.msg import Time as BuiltinTime

from geometry_msgs.msg import PoseStamped, TwistStamped
from robot_interfaces.msg import RobotObservation
from sensor_msgs.msg import CameraInfo, Image, JointState

from service_interfaces.msg import GetAngleAct1


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

        # ---- Parameters (Inspire) ----
        self.hand_angle_topic = self.declare_parameter(
            "hand_angle_topic", "/angle_data"
        ).value
        self.hand_finger_id_order = self.declare_parameter(
            "hand_finger_id_order", [0, 1, 2, 3, 4, 5]
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

        qos = SensorDataQoS()

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
        self.last_hand_position: Optional[List[float]] = None
        self.last_hand_velocity: Optional[List[float]] = None
        self.last_hand_time: Optional[Time] = None

        self._last_hand_has_value = False

        self.create_subscription(GetAngleAct1, self.hand_angle_topic, self._on_hand_angle, qos)

        # ---- Subscribers (RealSense) ----
        self.last_rgb_image: Optional[Image] = None
        self.last_depth_image: Optional[Image] = None
        self.last_camera_info: Optional[CameraInfo] = None
        self.last_rs_stamp: Optional[Time] = None

        self.create_subscription(Image, self.rs_rgb_image_topic, self._on_rgb_image, qos)
        self.create_subscription(Image, self.rs_depth_image_topic, self._on_depth_image, qos)
        self.create_subscription(
            CameraInfo, self.rs_camera_info_topic, self._on_camera_info, qos
        )

        # ---- Publisher ----
        self.observation_pub = self.create_publisher(RobotObservation, self.observation_topic, 10)

        self.timer = self.create_timer(1.0 / max(0.1, self.publish_rate_hz), self._on_timer)

        self.get_logger().info(f"Publishing: {self.observation_topic}")
        self.get_logger().info(f"FR3 JointState candidates: {self.arm_joint_state_topic_candidates}")

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

        # finger_id -> angle
        angle_by_id: Dict[int, float] = {}
        n = min(len(msg.finger_ids), len(msg.angles))
        for i in range(n):
            angle_by_id[int(msg.finger_ids[i])] = float(msg.angles[i])

        # Build ordered position array with NaN for missing.
        pos: List[float] = [math.nan for _ in self.hand_finger_id_order]
        for j, fid in enumerate(self.hand_finger_id_order):
            if int(fid) in angle_by_id:
                pos[j] = angle_by_id[int(fid)]

        # Velocity estimation by finite difference.
        vel = [0.0 for _ in self.hand_finger_id_order]
        if self._last_hand_has_value and self.last_hand_time is not None and len(pos) == len(self.last_hand_position or []):
            dt = (now - self.last_hand_time).nanoseconds / 1e9
            if dt > 1e-6:
                for j in range(len(pos)):
                    if math.isfinite(pos[j]) and math.isfinite(self.last_hand_position[j]):
                        vel[j] = (pos[j] - self.last_hand_position[j]) / dt

        self.last_hand_position = pos
        self.last_hand_velocity = vel
        self.last_hand_time = now
        self._last_hand_has_value = True

    def _on_rgb_image(self, msg: Image) -> None:
        self.last_rgb_image = msg
        self.last_rs_stamp = _time_from_msg(self, msg.header.stamp)

    def _on_depth_image(self, msg: Image) -> None:
        self.last_depth_image = msg
        self.last_rs_stamp = _time_from_msg(self, msg.header.stamp)

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self.last_camera_info = msg
        self.last_rs_stamp = _time_from_msg(self, msg.header.stamp)

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

    @staticmethod
    def _max_time(times: List[Optional[Time]]) -> Optional[Time]:
        valid = [t for t in times if t is not None]
        if not valid:
            return None
        return max(valid, key=lambda x: x.nanoseconds)

    # ----------------- Timer -----------------
    def _on_timer(self) -> None:
        obs = RobotObservation()

        stamp = self._max_time(
            [
                self.last_arm_joint_stamp,
                self.last_arm_pose_stamp,
                self.last_arm_twist_stamp,
                self.last_hand_time,
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

        if self._last_hand_has_value and self.last_hand_position is not None and self.last_hand_velocity is not None:
            obs.hand_joint_position = list(self.last_hand_position)
            obs.hand_joint_velocity = list(self.last_hand_velocity)

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

