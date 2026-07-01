"""Evaluation helpers used by the README, tests, and demo notebook."""

from __future__ import annotations

from typing import Callable, Iterable, List, Tuple

from .agent import (
    FRAGILITY_LEVELS,
    ReactiveAgent,
    cargo_top_speed,
    fuzzy_informed_heuristic,
    fuzzy_speed,
)
from .fuzzy import FuzzySpeedController
from .map_model import DEFAULT_GOAL, DEFAULT_START, BoxHillDeliveryMap
from .planner import HybridPlanner, SpeedFunction, constant_speed


def route_to_string(path: Iterable[str] | None) -> str:
    return " -> ".join(path or [])


def create_system() -> Tuple[BoxHillDeliveryMap, FuzzySpeedController, HybridPlanner, ReactiveAgent]:
    env = BoxHillDeliveryMap()
    env.apply_school_zone()
    controller = FuzzySpeedController()
    planner = HybridPlanner(env)
    agent = ReactiveAgent(env, controller, planner)
    return env, controller, planner, agent


def admissibility_audit(
    env: BoxHillDeliveryMap,
    planner: HybridPlanner,
    goal: str,
    speed_fn: SpeedFunction,
    h_fn: Callable[[str, str], float],
    tolerance: float = 1e-9,
) -> List[dict]:
    rows = []
    for node in env.roads_km:
        if node == goal:
            continue
        estimate = h_fn(node, goal)
        truth = planner.uniform_cost(node, goal, speed_fn).time_min
        rows.append({
            "node": node,
            "h_min": round(estimate, 4),
            "optimal_remaining_min": round(truth, 4),
            "admissible": estimate <= truth + tolerance,
        })
    return rows


def bad_school_zone_heuristic(env: BoxHillDeliveryMap):
    return lambda node, goal: env.straight_line_km(node, goal) / 40.0 * 60.0


def master_comparison(start: str = DEFAULT_START, goal: str = DEFAULT_GOAL) -> List[dict]:
    env, controller, planner, agent = create_system()
    rows = []
    baseline = planner.astar(start, goal, constant_speed(env.V_MAX))
    rows.append({
        "case": "Baseline constant speed",
        "route": route_to_string(baseline.path),
        "distance_km": round(baseline.dist_km, 2),
        "time_min": round(baseline.time_min, 2),
        "planning_nodes": baseline.nodes_expanded,
        "rerouted": False,
    })
    for label, fragility in FRAGILITY_LEVELS.items():
        heuristic = fuzzy_informed_heuristic(env, cargo_top_speed(controller, fragility))
        result = planner.astar(start, goal, fuzzy_speed(env, controller, fragility), h_fn=heuristic)
        rows.append({
            "case": f"Fuzzy A*, {label}",
            "route": route_to_string(result.path),
            "distance_km": round(result.dist_km, 2),
            "time_min": round(result.time_min, 2),
            "planning_nodes": result.nodes_expanded,
            "rerouted": False,
        })
    for label, fragility in FRAGILITY_LEVELS.items():
        journey = agent.run(start, goal, fragility)
        rows.append({
            "case": f"Reactive replanning, {label}",
            "route": route_to_string(journey.actual_path),
            "distance_km": round(journey.actual_km, 2),
            "time_min": round(journey.actual_min, 2),
            "planning_nodes": journey.total_planning_nodes,
            "rerouted": journey.rerouted,
        })
    return rows


def sensitivity_analysis(levels=(0.0, 2.0, 5.0, 8.0, 10.0), start: str = DEFAULT_START, goal: str = DEFAULT_GOAL) -> List[dict]:
    env, controller, planner, agent = create_system()
    rows = []
    for fragility in levels:
        heuristic = fuzzy_informed_heuristic(env, cargo_top_speed(controller, fragility))
        plan = planner.astar(start, goal, fuzzy_speed(env, controller, fragility), h_fn=heuristic)
        journey = agent.run(start, goal, fragility)
        rows.append({
            "fragility": fragility,
            "planned_route": route_to_string(plan.path),
            "planned_time_min": round(plan.time_min, 2),
            "actual_time_min": round(journey.actual_min, 2),
            "rerouted": journey.rerouted,
            "avg_planned_speed_kmh": round(plan.dist_km / plan.time_min * 60.0, 1),
        })
    return rows


def phase_aware_breakdown(journey, env: BoxHillDeliveryMap, controller: FuzzySpeedController) -> List[dict]:
    rows = []
    prefix_edges = journey.trigger_after - 1
    for i in range(len(journey.actual_path) - 1):
        u, v = journey.actual_path[i], journey.actual_path[i + 1]
        distance = env.edge_km(u, v)
        bumpiness = env.edge_bumpiness(u, v)
        capped = frozenset((u, v)) in env.capped_edges
        if i < prefix_edges:
            phase = "committed pre-zone"
            speed = controller.safe_speed(journey.fragility, bumpiness)
            source = "fuzzy"
        elif capped:
            phase = "replanned school zone"
            speed = 40.0
            source = "school-zone cap"
        else:
            phase = "replanned school zone"
            speed = controller.safe_speed(journey.fragility, bumpiness)
            source = "fuzzy"
        rows.append({
            "phase": phase,
            "segment": f"{u} -> {v}",
            "distance_km": round(distance, 2),
            "bumpiness": bumpiness,
            "speed_source": source,
            "speed_kmh": round(speed, 1),
            "time_min": distance / speed * 60.0,
        })
    return rows
