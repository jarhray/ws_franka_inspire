from __future__ import annotations

from typing import Dict, Type

from rclpy.node import Node

from policy_manager.act_dex_0_policy_adapter import ACTDex0PolicyAdapter
from policy_manager.act_exp3_0_policy_adapter import ACTExp30PolicyAdapter
from policy_manager.base_policy import BasePolicy
from policy_manager.dummy_policy import DummyPolicy


PolicyClass = Type[BasePolicy]


POLICY_REGISTRY: Dict[str, PolicyClass] = {
    "dummy": DummyPolicy,
    "act_dex_0": ACTDex0PolicyAdapter,
    "act_exp3_0": ACTExp30PolicyAdapter,
}


def available_policy_types() -> tuple[str, ...]:
    """Return policy names accepted by the registry."""
    return tuple(sorted(POLICY_REGISTRY))


def create_policy(policy_type: str, node: Node) -> BasePolicy:
    """Create a policy adapter from a registered YAML-friendly policy name."""
    policy_class = POLICY_REGISTRY.get(policy_type)
    if policy_class is None:
        available = ", ".join(available_policy_types())
        raise RuntimeError(
            f"Unknown policy_type={policy_type}. Available: {available}."
        )

    return policy_class.from_node(node)
