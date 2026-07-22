from __future__ import annotations

from typing import Dict, Type

from rclpy.node import Node

from policy_manager.act_exp1_1_policy_adapter import ACTExp11PolicyAdapter
from policy_manager.act_exp1_2_policy_adapter import ACTExp12PolicyAdapter
from policy_manager.act_exp1_3_policy_adapter import ACTExp13PolicyAdapter
from policy_manager.act_exp1_policy_adapter import ACTExp1PolicyAdapter
from policy_manager.act_exp3_0_det_policy_adapter import ACTExp30DetPolicyAdapter
from policy_manager.act_exp3_0_policy_adapter import ACTExp30PolicyAdapter
from policy_manager.act_exp3_1_policy_adapter import ACTExp31PolicyAdapter
from policy_manager.act_exp3_2_policy_adapter import ACTExp32PolicyAdapter
from policy_manager.act_exp3_5_det_policy_adapter import ACTExp35DetPolicyAdapter
from policy_manager.act_policy_adapter import ACTPolicyAdapter
from policy_manager.base_policy import BasePolicy
from policy_manager.dummy_policy import DummyPolicy
from policy_manager.pi05_exp1_3_lora_policy_adapter import (
    PI05Exp13LoraPolicyAdapter,
)


PolicyClass = Type[BasePolicy]


POLICY_REGISTRY: Dict[str, PolicyClass] = {
    "dummy": DummyPolicy,
    "act": ACTPolicyAdapter,
    "act_exp1": ACTExp1PolicyAdapter,
    "act_exp1_1": ACTExp11PolicyAdapter,
    "act_exp1_2": ACTExp12PolicyAdapter,
    "act_exp1_3": ACTExp13PolicyAdapter,
    "act_exp3_0": ACTExp30PolicyAdapter,
    "act_exp3_0_det": ACTExp30DetPolicyAdapter,
    "act_exp3_5_det": ACTExp35DetPolicyAdapter,
    "act_exp3_1": ACTExp31PolicyAdapter,
    "act_exp3_2": ACTExp32PolicyAdapter,
    "pi05_exp1_3_lora": PI05Exp13LoraPolicyAdapter,
}

_DUMMY_FALLBACK_POLICY_TYPES = {"bc", "vla"}


def available_policy_types() -> tuple[str, ...]:
    """Return policy names accepted by the registry."""
    return tuple(sorted((*POLICY_REGISTRY.keys(), *_DUMMY_FALLBACK_POLICY_TYPES)))


def create_policy(policy_type: str, node: Node) -> BasePolicy:
    """Create a policy adapter from a registered YAML-friendly policy name."""
    if policy_type in _DUMMY_FALLBACK_POLICY_TYPES:
        node.get_logger().warn(
            f"policy_type={policy_type} not integrated; using dummy policy parameters."
        )
        return DummyPolicy.from_node(node)

    policy_class = POLICY_REGISTRY.get(policy_type)
    if policy_class is None:
        available = ", ".join(available_policy_types())
        raise RuntimeError(f"Unknown policy_type={policy_type}. Available: {available}.")

    return policy_class.from_node(node)
