"""Regression tests for optimal, deterministic A* behaviour."""

from __future__ import annotations

from hybrid_delivery_router import Node, RoadNetwork, RoadSegment, ScenarioMap, constant_speed
from hybrid_delivery_router.planner import HybridPlanner


def test_astar_reopens_a_closed_node_for_an_inconsistent_admissible_heuristic() -> None:
    network = RoadNetwork(
        identifier="reopen",
        name="Reopen regression",
        nodes=(
            Node("S", (0.0, 0.0), "Start"),
            Node("A", (1.0, 0.0), "A"),
            Node("B", (0.0, -1.0), "B"),
            Node("G", (2.0, 0.0), "Goal"),
        ),
        roads=(
            RoadSegment("S", "A", 2.0, 1.0),
            RoadSegment("S", "B", 1.0, 1.0),
            RoadSegment("B", "A", 0.5, 1.0),
            RoadSegment("A", "G", 2.0, 1.0),
            RoadSegment("B", "G", 100.0, 1.0),
        ),
    )
    planner = HybridPlanner(ScenarioMap(network))
    heuristic_values = {"S": 3.5, "A": 0.0, "B": 2.5, "G": 0.0}

    def heuristic(node: str, _goal: str) -> float:
        return heuristic_values[node]

    astar = planner.astar("S", "G", constant_speed(60.0), heuristic)
    uniform_cost = planner.uniform_cost("S", "G", constant_speed(60.0))

    assert astar.path == ("S", "B", "A", "G")
    assert astar.time_min == uniform_cost.time_min == 3.5
    assert astar.statistics.nodes_reopened == 1
