"""Focused validation tests for scenario and routing inputs."""

from __future__ import annotations

from dataclasses import replace

import pytest

from hybrid_delivery_router import (
    NetworkValidationError,
    Node,
    RoadNetwork,
    RoadSegment,
    ScenarioMap,
    UnknownNodeError,
    constant_speed,
)
from hybrid_delivery_router.planner import HybridPlanner
from hybrid_delivery_router.scenarios import validate_network


def sample_network() -> RoadNetwork:
    return RoadNetwork(
        identifier="sample",
        name="Sample",
        nodes=(
            Node("A", (0.0, 0.0), "Start"),
            Node("B", (1.0, 0.0), "Destination"),
        ),
        roads=(RoadSegment("A", "B", 0.5, 3.0),),
        default_start="A",
        default_goal="B",
    )


@pytest.mark.parametrize(
    "network",
    (
        replace(sample_network(), nodes=(Node("A", (0.0, 0.0), "A"), Node("A", (1.0, 0.0), "B"))),
        replace(
            sample_network(),
            nodes=(Node("A", (float("nan"), 0.0), "A"), Node("B", (1.0, 0.0), "B")),
        ),
        replace(sample_network(), roads=(RoadSegment("A", "B", 0.0, 3.0),)),
        replace(sample_network(), roads=(RoadSegment("A", "B", 0.5, 11.0),)),
        replace(sample_network(), roads=(RoadSegment("A", "C", 0.5, 3.0),)),
        replace(
            sample_network(),
            roads=(RoadSegment("A", "B", 0.5, 3.0), RoadSegment("B", "A", 0.5, 3.0)),
        ),
    ),
)
def test_network_validation_rejects_malformed_data(network: RoadNetwork) -> None:
    with pytest.raises(NetworkValidationError):
        validate_network(network)


def test_map_uses_domain_errors_for_invalid_inputs() -> None:
    road_map = ScenarioMap(sample_network())
    planner = HybridPlanner(road_map)

    with pytest.raises(UnknownNodeError):
        planner.astar("missing", "B", constant_speed())
    with pytest.raises(UnknownNodeError):
        road_map.validate_path(("A", "missing"))
    with pytest.raises(ValueError, match="positive finite"):
        planner.edge_time_min(1.0, 0.0)
    with pytest.raises(ValueError, match="between 0 and 1"):
        road_map.apply_school_zone(1.1)


def test_disconnected_network_returns_an_unreachable_route() -> None:
    disconnected = RoadNetwork(
        identifier="disconnected",
        name="Disconnected",
        nodes=(Node("A", (0.0, 0.0), "A"), Node("B", (1.0, 0.0), "B"), Node("C", (2.0, 0.0), "C")),
        roads=(RoadSegment("A", "B", 0.5, 2.0),),
    )
    route = HybridPlanner(ScenarioMap(disconnected)).astar("A", "C", constant_speed())
    assert route.path is None
