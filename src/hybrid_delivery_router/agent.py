"""Hybrid speed models and reactive replanning over typed route results."""

from __future__ import annotations

import math
from typing import Protocol

from .domain import JourneyResult
from .fuzzy import FuzzySpeedController
from .planner import HeuristicFunction, HybridPlanner, RoutingGraph, SpeedFunction

FRAGILITY_LEVELS = {"Robust (2)": 2.0, "Moderate (5)": 5.0, "Delicate (8)": 8.0}


class RideQualityGraph(Protocol):
    """Additional network surface required by fuzzy speed and disruption models."""

    def edge_bumpiness(self, start: str, end: str) -> float: ...

    def apply_school_zone(self) -> None: ...

    def set_school_zone(self, active: bool) -> None: ...

    def is_capped(self, start: str, end: str) -> bool: ...

    @property
    def capped_edges(self) -> set[frozenset[str]]: ...


def fuzzy_speed(
    network: RideQualityGraph, controller: FuzzySpeedController, fragility: float
) -> SpeedFunction:
    """Return the fuzzy safe-speed model for a cargo profile."""

    return lambda start, end: controller.safe_speed(fragility, network.edge_bumpiness(start, end))


def schoolzone_speed(
    network: RideQualityGraph, controller: FuzzySpeedController, fragility: float
) -> SpeedFunction:
    """Return a speed model that honours the original school-zone demonstration."""

    def speed(start: str, end: str) -> float:
        if network.is_capped(start, end):
            return 40.0
        return controller.safe_speed(fragility, network.edge_bumpiness(start, end))

    return speed


def cargo_top_speed(controller: FuzzySpeedController, fragility: float) -> float:
    """Return the optimistic speed upper bound used by the fuzzy heuristic."""

    return controller.safe_speed(fragility, 0.0)


def fuzzy_informed_heuristic(network: RoutingGraph, top_speed_kmh: float) -> HeuristicFunction:
    """Build an admissible travel-time lower bound from a cargo speed cap."""

    return lambda node, goal: network.straight_line_km(node, goal) / top_speed_kmh * 60.0


JourneyRecord = JourneyResult


class ReactiveAgent:
    """Plan, activate a speed cap, and replan while retaining the driven prefix."""

    def __init__(
        self,
        network: RideQualityGraph,
        controller: FuzzySpeedController,
        planner: HybridPlanner,
    ) -> None:
        self.network = network
        self.env = network
        self.controller = controller
        self.planner = planner

    def run(self, start: str, goal: str, fragility: float) -> JourneyResult:
        if not self.network.capped_edges:
            self.network.apply_school_zone()

        heuristic = fuzzy_informed_heuristic(
            self.planner.network, cargo_top_speed(self.controller, fragility)
        )
        normal_speed = fuzzy_speed(self.network, self.controller, fragility)
        plan = self.planner.astar(start, goal, normal_speed, h_fn=heuristic)
        if plan.path is None:
            raise RuntimeError("No initial route found")

        trigger_after = math.ceil(0.20 * len(plan.path))
        committed = plan.path[:trigger_after]
        current = committed[-1]

        self.network.set_school_zone(True)
        try:
            capped_speed = schoolzone_speed(self.network, self.controller, fragility)
            replan = self.planner.astar(current, goal, capped_speed, h_fn=heuristic)
        finally:
            self.network.set_school_zone(False)

        if replan.path is None:
            raise RuntimeError("No replanned route found")

        actual_path = committed[:-1] + replan.path
        prefix_km = self.planner.path_km(committed)
        prefix_min = self.planner.path_time_min(committed, normal_speed)

        return JourneyResult(
            fragility=fragility,
            planned_path=plan.path,
            planned_km=plan.distance_km,
            planned_min=plan.time_min,
            planned_nodes=plan.nodes_expanded,
            trigger_after=trigger_after,
            current_node=current,
            actual_path=actual_path,
            actual_km=prefix_km + replan.distance_km,
            actual_min=prefix_min + replan.time_min,
            replan_nodes=replan.nodes_expanded,
            rerouted=actual_path != plan.path,
        )
