"""Tests for optional typed, explainable A* traces."""

from __future__ import annotations

from hybrid_delivery_router import BoxHillDeliveryMap, constant_speed, summarize_trace
from hybrid_delivery_router.planner import HybridPlanner


def test_search_trace_is_optional_and_contains_explanatory_events() -> None:
    planner = HybridPlanner(BoxHillDeliveryMap())
    without_trace = planner.astar("N1", "N18", constant_speed())
    with_trace = planner.astar("N1", "N18", constant_speed(), record_trace=True)

    assert without_trace.trace == ()
    kinds = {event.kind for event in with_trace.trace}
    assert {"select", "consider", "relax", "reject", "goal", "reconstruct"} <= kinds
    assert summarize_trace(with_trace.trace).startswith("Selected ")
