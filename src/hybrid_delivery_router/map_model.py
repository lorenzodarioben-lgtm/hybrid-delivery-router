"""Graph adapter for packaged road-network scenarios."""

from __future__ import annotations

import math
import random

from .domain import InvalidRouteError, NetworkValidationError, RoadNetwork, UnknownNodeError
from .scenarios import DEFAULT_SCENARIO_ID, load_scenario, validate_network

RNG_SEED = 215
DEFAULT_START = "N1"
DEFAULT_GOAL = "N18"


class ScenarioMap:
    """Expose a scenario as the deterministic graph interface used by the planner."""

    V_MAX = 100.0

    def __init__(self, network: RoadNetwork) -> None:
        validate_network(network)
        self.network = network
        self.coords = {node.identifier: node.coordinates for node in network.nodes}
        self.landmarks = {node.identifier: node.label for node in network.nodes}
        self.roads_km: dict[str, dict[str, float]] = {node.identifier: {} for node in network.nodes}
        self.bumpiness: dict[frozenset[str], float] = {}
        for road in network.roads:
            self.roads_km[road.start][road.end] = road.distance_km
            self.roads_km[road.end][road.start] = road.distance_km
            self.bumpiness[frozenset((road.start, road.end))] = road.bumpiness
        for node, neighbours in self.roads_km.items():
            self.roads_km[node] = dict(sorted(neighbours.items()))
        self._validate_reciprocal_roads()
        self.km_per_grid = self._minimum_km_per_grid()
        self.school_zone_active = False
        self.capped_edges: set[frozenset[str]] = set()

    def _validate_reciprocal_roads(self) -> None:
        for start, neighbours in self.roads_km.items():
            for end, distance in neighbours.items():
                if start not in self.roads_km.get(end, {}):
                    raise NetworkValidationError(f"Road {start!r} -> {end!r} is not reciprocal")
                if self.roads_km[end][start] != distance:
                    raise NetworkValidationError(
                        f"Road {start!r} -> {end!r} has mismatched distance"
                    )
                if frozenset((start, end)) not in self.bumpiness:
                    raise NetworkValidationError(
                        f"Road {start!r} -> {end!r} has no bumpiness metadata"
                    )

    def _minimum_km_per_grid(self) -> float:
        ratios = (
            distance / math.dist(self.coords[start], self.coords[end])
            for start, neighbours in self.roads_km.items()
            for end, distance in neighbours.items()
            if self.coords[start] != self.coords[end]
        )
        return min(ratios)

    def num_edges(self) -> int:
        return sum(len(neighbours) for neighbours in self.roads_km.values()) // 2

    def edge_keys(self) -> list[frozenset[str]]:
        return sorted(self.bumpiness, key=lambda edge: tuple(sorted(edge)))

    def edge_list(self) -> list[tuple[str, str]]:
        return [self._ordered_edge(edge) for edge in self.edge_keys()]

    @staticmethod
    def _ordered_edge(edge: frozenset[str]) -> tuple[str, str]:
        first, second = sorted(edge)
        return first, second

    def neighbours(self, node: str) -> list[str]:
        self._require_node(node)
        return list(self.roads_km[node])

    def edge_km(self, start: str, end: str) -> float:
        self._require_edge(start, end)
        return self.roads_km[start][end]

    def edge_bumpiness(self, start: str, end: str) -> float:
        self._require_edge(start, end)
        return self.bumpiness[frozenset((start, end))]

    def straight_line_km(self, start: str, end: str) -> float:
        self._require_node(start)
        self._require_node(end)
        return self.km_per_grid * math.dist(self.coords[start], self.coords[end])

    def apply_school_zone(self, fraction: float = 0.60, seed: int = RNG_SEED) -> None:
        if not math.isfinite(fraction) or not 0 <= fraction <= 1:
            raise ValueError("School-zone fraction must be finite and between 0 and 1")
        count = round(fraction * len(self.edge_keys()))
        self.capped_edges = set(random.Random(seed).sample(self.edge_keys(), count))

    def set_school_zone(self, active: bool) -> None:
        self.school_zone_active = active

    def is_capped(self, start: str, end: str) -> bool:
        return self.school_zone_active and frozenset((start, end)) in self.capped_edges

    def is_connected(self) -> bool:
        if not self.roads_km:
            return False
        first = next(iter(self.roads_km))
        seen, stack = {first}, [first]
        while stack:
            current = stack.pop()
            for neighbour in self.neighbours(current):
                if neighbour not in seen:
                    seen.add(neighbour)
                    stack.append(neighbour)
        return len(seen) == len(self.roads_km)

    def validate_path(self, path: tuple[str, ...] | list[str]) -> None:
        if not path:
            raise InvalidRouteError("A route path must contain at least one node")
        for node in path:
            self._require_node(node)
        for start, end in zip(path, path[1:]):
            self._require_edge(start, end)

    def _require_node(self, node: str) -> None:
        if node not in self.roads_km:
            raise UnknownNodeError(f"Unknown node: {node!r}")

    def _require_edge(self, start: str, end: str) -> None:
        self._require_node(start)
        self._require_node(end)
        if end not in self.roads_km[start]:
            raise InvalidRouteError(f"No road exists between {start!r} and {end!r}")

    def summary(self) -> dict[str, object]:
        distances = [
            distance for neighbours in self.roads_km.values() for distance in neighbours.values()
        ]
        return {
            "id": self.network.identifier,
            "nodes": len(self.roads_km),
            "edges": self.num_edges(),
            "distance_min_km": min(distances),
            "distance_max_km": max(distances),
            "km_per_grid": round(self.km_per_grid, 3),
            "connected": self.is_connected(),
        }


class BoxHillDeliveryMap(ScenarioMap):
    """Compatibility adapter for the default Box Hill-inspired synthetic scenario."""

    def __init__(self) -> None:
        super().__init__(load_scenario(DEFAULT_SCENARIO_ID))
