#!/usr/bin/env python3
"""Record key pipeline topics to rotating JSONL files."""

from __future__ import annotations

import json
import os
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node

from robot_interfaces.msg import ArmAction, HandAction, RobotObservation
from std_msgs.msg import String


def _stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class ExperimentLogger(Node):
    def __init__(self) -> None:
        super().__init__('experiment_logger')

        self._enabled = self.declare_parameter('enabled', True).get_parameter_value().bool_value
        self._log_dir = self.declare_parameter('log_dir', '~/.ros/experiment_logs').get_parameter_value().string_value
        self._run_name = self.declare_parameter('run_name', '').get_parameter_value().string_value
        self._flush_every_n = int(
            self.declare_parameter('flush_every_n', 20).get_parameter_value().integer_value
        )

        self._observation_topic = self.declare_parameter(
            'observation_topic', '/robot/observation'
        ).get_parameter_value().string_value
        self._arm_action_topic = self.declare_parameter(
            'arm_action_topic', '/robot/arm_action'
        ).get_parameter_value().string_value
        self._hand_action_topic = self.declare_parameter(
            'hand_action_topic', '/robot/hand_action'
        ).get_parameter_value().string_value
        self._arm_status_topic = self.declare_parameter(
            'arm_status_topic', '/robot/arm_execution/status'
        ).get_parameter_value().string_value
        self._arm_error_topic = self.declare_parameter(
            'arm_error_topic', '/robot/arm_execution/error'
        ).get_parameter_value().string_value

        self._files: Dict[str, object] = {}
        self._write_count: Dict[str, int] = {}
        self._session_id = self._make_session_id(self._run_name)

        if not self._enabled:
            self.get_logger().warn('experiment_logger is disabled by parameter.')
            return

        session_dir = Path(os.path.expanduser(self._log_dir)) / self._session_id
        session_dir.mkdir(parents=True, exist_ok=True)
        self._session_dir = session_dir

        self.create_subscription(
            RobotObservation, self._observation_topic, self._on_observation, 10
        )
        self.create_subscription(ArmAction, self._arm_action_topic, self._on_arm_action, 10)
        self.create_subscription(HandAction, self._hand_action_topic, self._on_hand_action, 10)
        self.create_subscription(String, self._arm_status_topic, self._on_arm_status, 10)
        self.create_subscription(String, self._arm_error_topic, self._on_arm_error, 10)

        self.get_logger().info(f'experiment_logger enabled; writing under {self._session_dir}')

    def _make_session_id(self, run_name: str) -> str:
        now = datetime.utcnow().strftime('%Y%m%d_%H%M%S')
        clean = run_name.strip().replace(' ', '_')
        if clean:
            return f'{now}_{clean}'
        return now

    def _jsonl_file(self, stream_name: str):
        fh = self._files.get(stream_name)
        if fh is not None:
            return fh
        file_path = self._session_dir / f'{stream_name}.jsonl'
        out = open(file_path, 'a', encoding='utf-8')
        self._files[stream_name] = out
        self._write_count[stream_name] = 0
        return out

    def _write_jsonl(self, stream_name: str, payload: Dict) -> None:
        if not self._enabled:
            return
        fh = self._jsonl_file(stream_name)
        fh.write(json.dumps(payload, ensure_ascii=True) + '\n')
        self._write_count[stream_name] = self._write_count.get(stream_name, 0) + 1
        flush_every = max(1, self._flush_every_n)
        if self._write_count[stream_name] % flush_every == 0:
            fh.flush()

    def _base_event(self, msg_stamp=None) -> Dict:
        now = self.get_clock().now().to_msg()
        event = {'ros_time_ns': _stamp_to_ns(now)}
        if msg_stamp is not None:
            event['msg_time_ns'] = _stamp_to_ns(msg_stamp)
        return event

    def _on_observation(self, msg: RobotObservation) -> None:
        event = self._base_event(msg.header.stamp)
        event.update(
            {
                'ee_pose_frame': msg.ee_pose_frame,
                'arm_joint_position': list(msg.arm_joint_position),
                'arm_joint_velocity': list(msg.arm_joint_velocity),
                'hand_joint_position': list(msg.hand_joint_position),
                'hand_joint_velocity': list(msg.hand_joint_velocity),
                'hand_touch_finger_ids': list(msg.hand_touch.finger_ids),
                'hand_touch_normal_forces': list(msg.hand_touch.normal_forces),
                'hand_touch_tangential_forces': list(msg.hand_touch.tangential_forces),
                'rgb_stamp_ns': _stamp_to_ns(msg.rgb_image.header.stamp),
                'depth_stamp_ns': _stamp_to_ns(msg.depth_image.header.stamp),
                'camera_info_stamp_ns': _stamp_to_ns(msg.camera_info.header.stamp),
            }
        )
        self._write_jsonl('observation', event)

    def _on_arm_action(self, msg: ArmAction) -> None:
        event = self._base_event(msg.header.stamp)
        event.update(
            {
                'control_mode': int(msg.control_mode),
                'is_relative': bool(msg.is_relative),
                'reference_frame': msg.reference_frame,
                'joint_position': list(msg.joint_position),
                'joint_velocity': list(msg.joint_velocity),
                'duration_sec': float(msg.duration_sec),
                'cartesian_pose': {
                    'position': {
                        'x': float(msg.cartesian_pose.position.x),
                        'y': float(msg.cartesian_pose.position.y),
                        'z': float(msg.cartesian_pose.position.z),
                    },
                    'orientation': {
                        'x': float(msg.cartesian_pose.orientation.x),
                        'y': float(msg.cartesian_pose.orientation.y),
                        'z': float(msg.cartesian_pose.orientation.z),
                        'w': float(msg.cartesian_pose.orientation.w),
                    },
                },
                'cartesian_velocity': {
                    'linear': {
                        'x': float(msg.cartesian_velocity.linear.x),
                        'y': float(msg.cartesian_velocity.linear.y),
                        'z': float(msg.cartesian_velocity.linear.z),
                    },
                    'angular': {
                        'x': float(msg.cartesian_velocity.angular.x),
                        'y': float(msg.cartesian_velocity.angular.y),
                        'z': float(msg.cartesian_velocity.angular.z),
                    },
                },
            }
        )
        self._write_jsonl('arm_action', event)

    def _on_hand_action(self, msg: HandAction) -> None:
        event = self._base_event(msg.header.stamp)
        event.update(
            {
                'control_mode': int(msg.control_mode),
                'is_relative': bool(msg.is_relative),
                'joint_position': list(msg.joint_position),
                'joint_velocity': list(msg.joint_velocity),
                'duration_sec': float(msg.duration_sec),
            }
        )
        self._write_jsonl('hand_action', event)

    def _on_arm_status(self, msg: String) -> None:
        event = self._base_event(None)
        event['text'] = msg.data
        self._write_jsonl('arm_status', event)

    def _on_arm_error(self, msg: String) -> None:
        event = self._base_event(None)
        event['text'] = msg.data
        self._write_jsonl('arm_error', event)

    def destroy_node(self) -> bool:
        for fh in self._files.values():
            try:
                fh.flush()
                fh.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args: Optional[Tuple[str, ...]] = None) -> None:
    rclpy.init(args=args)
    node = ExperimentLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
