"""Hybrid speed models and reactive replanning agent."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .fuzzy import FuzzySpeedController
from .map_model import BoxHillDeliveryMap
from .planner import HybridPlanner, SpeedFunction

FRAGILITY_LEVELS = {"Robust (2)": 2.0, "Moderate (5)": 5.0, "Delicate (8)": 8.0}


def fuzzy_speed(
    env: BoxHillDeliveryMap, controller: FuzzySpeedController, fragility: float
) -> SpeedFunction:
    return lambda u, v: controller.safe_speed(fragility, env.edge_bumpiness(u, v))


def schoolzone_speed(
    env: BoxHillDeliveryMap, controller: FuzzySpeedController, fragility: float
) -> SpeedFunction:
    def speed(u: str, v: str) -> float:
        return (
            40.0
            if env.is_capped(u, v)
            else controller.safe_speed(fragility, env.edge_bumpiness(u, v))
        )

    return speed


def cargo_top_speed(controller: FuzzySpeedController, fragility: float) -> float:
    return controller.safe_speed(fragility, 0.0)


def fuzzy_informed_heuristic(env: BoxHillDeliveryMap, top_speed_kmh: float):
    return lambda node, goal: env.straight_line_km(node, goal) / top_speed_kmh * 60.0


@dataclass(frozen=True)
class JourneyRecord:
    fragility: float
    planned_path: list[str]
    planned_km: float
    planned_min: float
    planned_nodes: int
    trigger_after: int
    current_node: str
    actual_path: list[str]
    actual_km: float
    actual_min: float
    replan_nodes: int
    rerouted: bool

    @property
    def total_planning_nodes(self) -> int:
        return self.planned_nodes + self.replan_nodes


class ReactiveAgent:
    """Plan, sense a mid-route speed cap, and replan from the current node."""

    def __init__(
        self, env: BoxHillDeliveryMap, controller: FuzzySpeedController, planner: HybridPlanner
    ) -> None:
        self.env = env
        self.controller = controller
        self.planner = planner

    def run(self, start: str, goal: str, fragility: float) -> JourneyRecord:
        if not self.env.capped_edges:
            self.env.apply_school_zone()

        top_speed = cargo_top_speed(self.controller, fragility)
        heuristic = fuzzy_informed_heuristic(self.env, top_speed)
        normal_speed = fuzzy_speed(self.env, self.controller, fragility)
        plan = self.planner.astar(start, goal, normal_speed, h_fn=heuristic)
        if not plan.path:
            raise RuntimeError("No initial route found")

        trigger_after = math.ceil(0.20 * len(plan.path))
        committed = plan.path[:trigger_after]
        current = committed[-1]

        self.env.set_school_zone(True)
        try:
            capped_speed = schoolzone_speed(self.env, self.controller, fragility)
            replan = self.planner.astar(current, goal, capped_speed, h_fn=heuristic)
        finally:
            self.env.set_school_zone(False)

        if not replan.path:
            raise RuntimeError("No replanned route found")

        actual_path = committed[:-1] + replan.path
        prefix_km = self.planner.path_km(committed)
        prefix_min = self.planner.path_time_min(committed, normal_speed)

        return JourneyRecord(
            fragility=fragility,
            planned_path=plan.path,
            planned_km=plan.dist_km,
            planned_min=plan.time_min,
            planned_nodes=plan.nodes_expanded,
            trigger_after=trigger_after,
            current_node=current,
            actual_path=actual_path,
            actual_km=prefix_km + replan.dist_km,
            actual_min=prefix_min + replan.time_min,
            replan_nodes=replan.nodes_expanded,
            rerouted=actual_path != plan.path,
        )
