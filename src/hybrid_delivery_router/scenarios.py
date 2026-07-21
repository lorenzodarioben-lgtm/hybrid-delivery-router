"""Load version-controlled synthetic delivery-network scenarios."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

from .domain import NetworkValidationError, Node, RoadNetwork, RoadSegment, RoutingError

SCENARIO_DIRECTORY = Path(__file__).with_name("data")
DEFAULT_SCENARIO_ID = "box-hill-synthetic"


class ScenarioLoadError(RoutingError):
    """Raised when a scenario file cannot be found or decoded."""


def available_scenarios() -> tuple[str, ...]:
    """Return scenario identifiers in stable alphabetical order."""

    return tuple(path.stem for path in sorted(SCENARIO_DIRECTORY.glob("*.json")))


def scenario_path(scenario_id: str) -> Path:
    """Resolve a known scenario identifier to its packaged JSON definition."""

    candidate = SCENARIO_DIRECTORY / f"{scenario_id}.json"
    if not candidate.is_file():
        available = ", ".join(available_scenarios())
        raise ScenarioLoadError(
            f"Unknown scenario {scenario_id!r}. Available scenarios: {available}"
        )
    return candidate


def load_scenario(scenario_id: str = DEFAULT_SCENARIO_ID) -> RoadNetwork:
    """Load a portable road-network definition from a local JSON scenario file."""

    try:
        payload = json.loads(scenario_path(scenario_id).read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ScenarioLoadError(f"Scenario {scenario_id!r} is not valid JSON") from error
    if not isinstance(payload, dict):
        raise ScenarioLoadError(f"Scenario {scenario_id!r} must contain a JSON object")
    return _network_from_payload(payload, scenario_id)


def _network_from_payload(payload: dict[str, Any], requested_id: str) -> RoadNetwork:
    try:
        nodes = tuple(
            Node(
                identifier=str(item["id"]),
                coordinates=(float(item["coordinates"][0]), float(item["coordinates"][1])),
                label=str(item["label"]),
            )
            for item in payload["nodes"]
        )
        roads = tuple(
            RoadSegment(
                start=str(item["start"]),
                end=str(item["end"]),
                distance_km=float(item["distance_km"]),
                bumpiness=float(item["bumpiness"]),
            )
            for item in payload["roads"]
        )
        network = RoadNetwork(
            identifier=str(payload["id"]),
            name=str(payload["name"]),
            description=str(payload.get("description", "")),
            default_start=_optional_string(payload.get("default_start")),
            default_goal=_optional_string(payload.get("default_goal")),
            nodes=nodes,
            roads=roads,
        )
    except (KeyError, IndexError, TypeError, ValueError) as error:
        raise ScenarioLoadError(f"Scenario {requested_id!r} has an invalid structure") from error
    validate_network(network)
    return network


def _optional_string(value: object) -> str | None:
    return None if value is None else str(value)


def validate_network(network: RoadNetwork) -> None:
    """Validate graph structure and metadata without requiring global reachability."""

    identifiers = [node.identifier for node in network.nodes]
    if not identifiers or len(identifiers) != len(set(identifiers)):
        raise NetworkValidationError("Nodes must have unique identifiers")
    for node in network.nodes:
        if not node.identifier:
            raise NetworkValidationError("Node identifiers must not be empty")
        if not all(math.isfinite(value) for value in node.coordinates):
            raise NetworkValidationError(f"Node {node.identifier!r} has non-finite coordinates")

    node_ids = set(identifiers)
    endpoints = (
        ("default_start", network.default_start),
        ("default_goal", network.default_goal),
    )
    for endpoint_name, endpoint in endpoints:
        if endpoint is not None and endpoint not in node_ids:
            raise NetworkValidationError(f"{endpoint_name} {endpoint!r} is not a node")

    edges: set[tuple[str, str]] = set()
    for road in network.roads:
        if road.start == road.end or road.start not in node_ids or road.end not in node_ids:
            raise NetworkValidationError(
                f"Road {road.start!r} -> {road.end!r} has invalid endpoints"
            )
        if not math.isfinite(road.distance_km) or road.distance_km <= 0:
            raise NetworkValidationError(f"Road {road.edge!r} needs a positive finite distance")
        if not math.isfinite(road.bumpiness) or not 0 <= road.bumpiness <= 10:
            raise NetworkValidationError(f"Road {road.edge!r} needs bumpiness within 0 to 10")
        if road.edge in edges:
            raise NetworkValidationError(f"Road {road.edge!r} is defined more than once")
        edges.add(road.edge)
