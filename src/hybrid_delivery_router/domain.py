"""Immutable domain contracts for explainable delivery routing."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

Edge = tuple[str, str]
SearchEventKind = Literal[
    "select",
    "consider",
    "relax",
    "reject",
    "reopen",
    "goal",
    "reconstruct",
]


class RoutingError(ValueError):
    """Base error for invalid routing data or requests."""


class NetworkValidationError(RoutingError):
    """Raised when a road network violates its data contract."""


class UnknownNodeError(RoutingError):
    """Raised when a requested node does not belong to a network."""


class InvalidRouteError(RoutingError):
    """Raised when a route references an invalid road segment."""


class UnreachableRouteError(RoutingError):
    """Raised when a route cannot be found between valid nodes."""


@dataclass(frozen=True, slots=True)
class Node:
    """A named road-network intersection in synthetic map coordinates."""

    identifier: str
    coordinates: tuple[float, float]
    label: str


@dataclass(frozen=True, slots=True)
class RoadSegment:
    """An undirected road with travel distance and ride-quality metadata."""

    start: str
    end: str
    distance_km: float
    bumpiness: float

    @property
    def edge(self) -> Edge:
        first, second = sorted((self.start, self.end))
        return first, second


@dataclass(frozen=True, slots=True)
class RoadNetwork:
    """A portable, ordered scenario definition independent of its data source."""

    identifier: str
    name: str
    nodes: tuple[Node, ...]
    roads: tuple[RoadSegment, ...]
    description: str = ""
    default_start: str | None = None
    default_goal: str | None = None


@dataclass(frozen=True, slots=True)
class CargoProfile:
    """Cargo sensitivity supplied to the fuzzy speed controller."""

    fragility: float
    label: str = "Custom cargo"


@dataclass(frozen=True, slots=True)
class RouteObjective:
    """Weights used to score a route; travel time remains the default objective."""

    travel_time: float = 1.0
    distance: float = 0.0
    cargo_risk: float = 0.0
    disruption_exposure: float = 0.0


@dataclass(frozen=True, slots=True)
class ScoreBreakdown:
    """The explainable, unit-preserving components of a route score."""

    travel_time: float
    distance: float = 0.0
    cargo_risk: float = 0.0
    disruption_exposure: float = 0.0
    total: float = 0.0


@dataclass(frozen=True, slots=True)
class SearchStatistics:
    """Deterministic accounting for a graph-search execution."""

    nodes_expanded: int
    max_frontier: int
    nodes_reopened: int = 0
    stale_entries_skipped: int = 0


@dataclass(frozen=True, slots=True)
class SearchEvent:
    """One typed event emitted by optional explainable search tracing."""

    kind: SearchEventKind
    order: int
    node: str
    g_min: float
    h_min: float
    f_min: float
    edge: Edge | None = None
    frontier_size: int = 0
    accepted: bool | None = None


@dataclass(frozen=True, slots=True)
class RoutePlan:
    """An immutable planned route and its measured search result."""

    path: tuple[str, ...] | None
    time_min: float
    distance_km: float
    statistics: SearchStatistics
    trace: tuple[SearchEvent, ...] = ()
    score: ScoreBreakdown | None = None

    @property
    def dist_km(self) -> float:
        """Compatibility name for the route distance."""

        return self.distance_km

    @property
    def nodes_expanded(self) -> int:
        """Compatibility accessor for search statistics."""

        return self.statistics.nodes_expanded

    @property
    def max_frontier(self) -> int:
        """Compatibility accessor for search statistics."""

        return self.statistics.max_frontier


@dataclass(frozen=True, slots=True)
class JourneySegment:
    """One travelled delivery segment, including the speed and phase used."""

    start: str
    end: str
    distance_km: float
    bumpiness: float
    speed_kmh: float
    travel_time_min: float
    disruption_id: str | None
    committed: bool


@dataclass(frozen=True, slots=True)
class JourneyResult:
    """A completed journey with original plan, committed prefix, and replanned tail."""

    fragility: float
    planned_path: tuple[str, ...]
    planned_km: float
    planned_min: float
    planned_nodes: int
    trigger_after: int
    current_node: str
    actual_path: tuple[str, ...]
    actual_km: float
    actual_min: float
    replan_nodes: int
    rerouted: bool
    segments: tuple[JourneySegment, ...] = field(default_factory=tuple)

    @property
    def total_planning_nodes(self) -> int:
        """Total nodes expanded by initial planning and replanning."""

        return self.planned_nodes + self.replan_nodes
