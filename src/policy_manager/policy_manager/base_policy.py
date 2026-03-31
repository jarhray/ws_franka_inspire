from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

from rclpy.node import Node
from robot_interfaces.msg import RobotObservation, WholeBodyAction


class BasePolicy(ABC):
    """统一的策略抽象基类。

    约束所有具体策略：
    - 通过 from_node 在 ROS Node 上声明并读取自己需要的参数
    - 输入：RobotObservation
    - 输出：WholeBodyAction（或在缺失观测等情况下返回 None）
    """

    @classmethod
    @abstractmethod
    def from_node(cls, node: Node) -> "BasePolicy":
        """使用给定 Node 的参数服务器构造策略实例。"""
        raise NotImplementedError

    @property
    @abstractmethod
    def control_hz(self) -> float:
        """策略自身希望运行的控制频率。"""
        raise NotImplementedError

    @abstractmethod
    def infer(self, obs: RobotObservation) -> Optional[WholeBodyAction]:
        """单步推理接口。"""
        raise NotImplementedError
